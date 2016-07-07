#include <pirate/motion_primitives.h>


////////////////////////////////////////////////////////////////////////////////
// ****************************** BASIC FUNCTIONS **************************** //
//     These functions are basic and DO NOT PUBLISH MESSAGES TO THE PIRATE     //
//                   They ONLY change the setpoint or control mode             //
/////////////////////////////////////////////////////////////////////////////////
 
 // GENERAL REFERENCES OF MAPPING //
 /* SETPOINT_MSG
setpoint_msg.setpoint[1] -> Module 21 - Front module 
setpoint_msg.setpoint[2] -> Module 22 - Bend module
setpoint_msg.setpoint[3] -> Module 23 - Bend module
setpoint_msg.setpoint[5] -> Module 25 - Rotational module
setpoint_msg.setpoint[6] -> Module 26 - Bend module
setpoint_msg.setpoint[7] -> Module 27 - Bend module 
setpoint_msg.setpoint[11] -> All Wheels (Used for drive function)
*/

/* CONTROL_MSG
control_msg.controlMode[1] -> Module 21 - Front module
control_msg.controlMode[2] -> Module 22 - Bend module
control_msg.controlMode[3] -> Module 23 - Bend module
control_msg.controlMode[5] -> Module 25 - Rotational module
control_msg.controlMode[6] -> Module 26 - Bend module
control_msg.controlMode[7] -> Module 27 - Bend module
*/

//Constructor
Motion_Primitives::Motion_Primitives(Setpoint& setpoint, Control& control):
setpoint_(setpoint),
control_(control)
 {
    ROS_INFO("Creating Motion Primitives class instance");
    ROS_INFO("Service joint state established.");
    joint_client = nh_.serviceClient<pirate_srv::joint_state>("jointstate");
}

//Destructor
Motion_Primitives::~Motion_Primitives(void){
    // empty destructor
}


// This function can be used for bend modules and the front module ---> NOT MEANT TO BE USED WITH THE CAMERAS
// Possible modules are 21, 22, 23, 26 and 27
void Motion_Primitives::moveBFModule(uint8_t moduleID, int setpoint) {
    //bool done= false;
    //ROS_INFO("Executing move BFModule");
    setpoint_.setElement(moduleID, Motor0, setpoint);
    //done=setpoint_.publish();  
}   

// This function is similar to moveBFmodule but specifically for the rotational module and the setpoint should be sent in radians 
void Motion_Primitives::rotateRad(double radians) {
    bool done= false;
    int16_t setpoint = round(radians*Conv_Rad_Setpoint);
    setpoint_.setElement(RotationModule, Motor0, setpoint);
    done=setpoint_.publish(); // publish rotate message
    } 

// This function can be used for bend modules, rotation module and the front module ---> NOT MEANT TO BE USED WITH THE CAMERAS
// Possible modules are 21, 22, 23, 25, 26 and 27
void Motion_Primitives::changeControlBFModule(uint8_t moduleID, uint8_t control_mode) {
   control_.setElement(moduleID, Motor0, control_mode);  
}

void Motion_Primitives::changeControlPanCam(uint8_t control_mode){
   control_.setElement(CameraModule, Motor1, control_mode); 
}

void Motion_Primitives::changeControlDrive(uint8_t control_mode)
{
    control_.setElement(Drive, Motor1, control_mode);
}


////////////////////////////////////////////////////////////////////////////////////////
// ******************************* ADVANCED FUNCTIONS ******************************* //
//            These functions use the basic motion primitive functions                //
//            WARNING: These functions WILL PUBLISH messages to the PIRATE            //
////////////////////////////////////////////////////////////////////////////////////////

// This function can be used to send mm/s velocity to the wheels, Directions: + forward, -backward
/*  MAPPING FOR THE CONTROL_MSG

*/
void Motion_Primitives::driveVel(int velocity) {
   /* conversion part goes here */
   /* The final setpoint sent to the PIRATE (in the setpoint_msgs[11] has to given in terms of the number of counts every 50Hz, i.e. counts/20ms) */
   /* Diameter of the wheels is 0.046m) 
   /*
   /*   m     1 rev     counts     1      counts
   /*  --- = ------- = -------- = ---- = -------- 
   /*   s    pi*diam     rev       50      20ms 
    */
   // setpoint is given in counts/20ms time (because of the mapping of the PID) 
   
   //check if velocity is not greater than +/- 46mm/s
   if (velocity < abs(Max_velocity)){
       int16_t setpoint = round((velocity/MM_per_Count)*(Conv_to_20ms));
       setpoint_.setElement(Drive, Motor1, setpoint);
   }
   else {
       setpoint_.setElement(Drive, Motor1, error_setpoint);
       ROS_INFO_STREAM("Setpoint is bigger than velocity limit 46mm/s");
   }   
}



/////////////////// FOR REAR /////////////////

// This function clamps the rear part of the PIRATE (affects bend modules 26 and 27) in a 110mm pipe
bool Motion_Primitives::clampRear110(void) {
    //local variable to hold result
    bool done= false;
    bool published = false;
    moveBFModule(BendModule26,Setpoint_CR_26); // write setpoint to clamp rear
    moveBFModule(BendModule27,Setpoint_CR_27); // write setpoint to clamp rear
    published= setpoint_.publish();  // publish the clamping setpoint
    while (!done) {
        if (joint_client.call(joint_srv)){
            if ((joint_srv.response.joint_state.position[4] >=Position_CR_26_110)  && (joint_srv.response.joint_state.position[5] <= Position_CR_27_110)){
                moveBFModule(BendModule26, Position_Stop_26); // stop module 26
                done=setpoint_.publish(); // publish new message to stop module 26 -> Lesser torque 
            }
        }
        else{
           ROS_ERROR("Failed to call service joint state");
        }
   }
   return done;
}

// This function clamps the rear part of the PIRATE (affects bend modules 26 and 27) in a 90mm pipe
bool Motion_Primitives::clampRear90(void) {
    //local variable to hold result
    bool published = false;
    bool done= false;
    moveBFModule(BendModule26,Setpoint_CR_26); // write setpoint to clamp rear
    moveBFModule(BendModule27,Setpoint_CR_27); // write setpoint to clamp rear
    published = setpoint_.publish();  // publish the clamping setpoint
    while (!done) {
        if (joint_client.call(joint_srv)){
            if ((joint_srv.response.joint_state.position[4] >=Position_CR_26_90)  && (joint_srv.response.joint_state.position[5] <= Position_CR_27_90)){
                 moveBFModule(BendModule26, Position_Stop_26); // stop module 26
                 done= setpoint_.publish(); // publish new message to stop module 26 -> Lesser torque
            }
        }
        else{
           ROS_ERROR("Failed to call service joint state");
        }
   }
   return done;
}

// This function PARTIALLY unclamps the rear part of the PIRATE (affects bend modules 26 and 27) in a 110mm pipe
bool Motion_Primitives::unclampPartialRear110(void) {
    //local variable to hold result
    bool published = false;
    bool done= false;
    moveBFModule(BendModule26,Setpoint_UNC_26); // write setpoint to clamp rear
    moveBFModule(BendModule27, Setpoint_UNC_27); // write setpoint to clamp rear
    published= setpoint_.publish();  // publish the clamping setpoint
    while (!done) {
         if (joint_client.call(joint_srv)){
            if ((joint_srv.response.joint_state.position[4] <= Position_UNCRP_26_110)   && (joint_srv.response.joint_state.position[5] >= Position_UNCRP_27_110)){ // is unclamp rear done?    
                moveBFModule(BendModule26, Position_Stop_26); // stop module 26
                done= setpoint_.publish(); // publish new message to stop module 26 -> Lesser torque
            }
         }
         else {
           ROS_ERROR("Failed to call service joint state");
        }
   }
   return done;
}

// This function PARTIALLY clamps the rear part of the PIRATE (affects bend modules 26 and 27) in a 90mm pipe
bool Motion_Primitives::unclampPartialRear90(void) {
    //local variable to hold result
    bool published = false;
    bool done= false;
    moveBFModule(BendModule26,Setpoint_UNC_26); // write setpoint to clamp rear
    moveBFModule(BendModule27,Setpoint_UNC_27); // write setpoint to clamp rear
    published= setpoint_.publish(); // publish the clamping setpoint
    while (!done) {
        if (joint_client.call(joint_srv)){
            if ((joint_srv.response.joint_state.position[4] <=Position_UNCRP_26_90)  && (joint_srv.response.joint_state.position[5] >=  Position_UNCRP_27_90)){ // is unclamped from rear done?
                moveBFModule(BendModule26, Position_Stop_26); // stop module 26
                done= setpoint_.publish(); // publish new message to stop module 26 -> Lesser torque
            }
         }
         else {
           ROS_ERROR("Failed to call service joint state");
        }
   }
   return done;
}
        

// This function FULLY unclamps the rear module (affects bend modules 26 and 27) in a 110mm pipe
bool Motion_Primitives::unclampFullRear(void) {
    //local variable to hold result
    bool published= false;
    bool done= false;
    moveBFModule(BendModule26,Setpoint_UNC_26); // write setpoint to clamp rear
    moveBFModule(BendModule27,Setpoint_UNC_27); // write setpoint to clamp rear
    published = setpoint_.publish();  // publish the clamping setpoint
    while (!done) {
        if (joint_client.call(joint_srv)){
            if ((joint_srv.response.joint_state.position[4] <= Position_UNCR_26)   && (joint_srv.response.joint_state.position[5] >= Position_UNCR_27)){ // is unclamp rear done?
                moveBFModule(BendModule26, Position_Stop_26); // stop module 26
                done= setpoint_.publish();// publish new message to stop module 26 -> Lesser torque
            }      
        }  
        else {
            ROS_ERROR("Failed to call service joint state");
        } 
   }
   return done;
}


/////////////////// FOR FRONT/////////////////

// This function clamps the rear part of the PIRATE (affects bend modules 22 and 23) in a 110mm pipe
bool Motion_Primitives::clampFront110(void) {
    //local variable to hold result
    bool published = false;
    bool done= false;
    moveBFModule(BendModule22, Setpoint_CF_22); // write setpoint to clamp front
    moveBFModule(BendModule23, Setpoint_CF_23); // write setpoint to clamp front
    published = setpoint_.publish(); // publish the clamping setpoint
    while (!done) {
        if (joint_client.call(joint_srv)){
            if ((joint_srv.response.joint_state.position[2] >=Position_CF_23_110)  && (joint_srv.response.joint_state.position[1] <= Position_CF_22_110)){
                moveBFModule(BendModule23, Position_Stop_23); // stop module 23
                done= setpoint_.publish(); // publish new message to stop module 23 -> Lesser torque
            }
        }
        else {
            ROS_ERROR("Failed to call service joint state");
        } 
   }
   return done;
}

// This function clamps the rear part of the PIRATE (affects bend modules 22 and 23) in a 90mm pipe
bool Motion_Primitives::clampFront90(void) {
    //local variable to hold result
    bool published = false;
    bool done= false;
    moveBFModule(BendModule22, Setpoint_CF_22); // write setpoint to clamp rear
    moveBFModule(BendModule23, Setpoint_CF_23); // write setpoint to clamp rear
    published = setpoint_.publish();  // publish the clamping setpoint
    while (!done) {
        if (joint_client.call(joint_srv)){
            if ((joint_srv.response.joint_state.position[2] >=Position_CF_23_90)  && (joint_srv.response.joint_state.position[1] <= Position_CF_22_90)){
                moveBFModule(BendModule23, Position_Stop_23); // stop module 23
                done= setpoint_.publish(); // publish new message to stop module 23 -> Lesser torque
            }
        }
        else {
            ROS_ERROR("Failed to call service joint state");
        } 
   }
   return done;
}

// This function PARTIALLY unclamps the rear part of the PIRATE (affects bend modules 22 and 23) in a 110mm pipe
bool Motion_Primitives::unclampPartialFront110(void) {
    //local variable to hold result
    bool published = false;
    bool done= false;
    moveBFModule(BendModule22,Setpoint_UNC_22); // write setpoint to clamp rear
    moveBFModule(BendModule23,Setpoint_UNC_22); // write setpoint to clamp rear
    published = setpoint_.publish();  // publish the clamping setpoint
    while (!done) {
        if (joint_client.call(joint_srv)){
            if ((joint_srv.response.joint_state.position[2] <= Position_UNCRP_23_110)   && (joint_srv.response.joint_state.position[1] >= Position_UNCRP_22_110)){ // is unclamp rear done?
                moveBFModule(BendModule23, Position_Stop_23); // stop module 23
                done= setpoint_.publish(); // publish new message to stop module 23 -> Lesser torque
            }
        }
        else {
            ROS_ERROR("Failed to call service joint state");
        } 
   }
   return done;
}

// This function PARTIALLY clamps the rear part of the PIRATE (affects bend modules 22 and 23) in a 90mm pipe
bool Motion_Primitives::unclampPartialFront90(void) {
    //local variable to hold result
    bool published = false;
    bool done= false;
    moveBFModule(BendModule22,Setpoint_UNC_22); // write setpoint to clamp rear
    moveBFModule(BendModule23,Setpoint_UNC_23); // write setpoint to clamp rear
    published= setpoint_.publish();  // publish the clamping setpoint
    while (!done) {
        if (joint_client.call(joint_srv)){
            if ((joint_srv.response.joint_state.position[2] <= Position_UNCRP_23_90)   && (joint_srv.response.joint_state.position[1] >= Position_UNCRP_22_90)){ // is unclamped from front done?
                moveBFModule(BendModule23, Position_Stop_23); // stop module 23
                done= setpoint_.publish(); // publish new message to stop module 23 -> Lesser torque
            }
        }
       else {
            ROS_ERROR("Failed to call service joint state");
        } 
   }
   return done;
}


// This function FULLY unclamps the rear module (affects bend modules 22 and 23) in a 110mm pipe
bool Motion_Primitives::unclampFullFront(void) {
    //local variable to hold result
    bool published= false;
    bool done= false;
    moveBFModule(BendModule22,Setpoint_UNC_22); // write setpoint to clamp rear
    moveBFModule(BendModule23, Setpoint_UNC_23); // write setpoint to clamp rear
    published= setpoint_.publish();  // publish the clamping setpoint
    while (!done) {
        if (joint_client.call(joint_srv)){
            if ((joint_srv.response.joint_state.position[2] <= Position_UNCR_23)   && (joint_srv.response.joint_state.position[1] >= Position_UNCR_22)){ // is unclamp front done?
                moveBFModule(BendModule23, Position_Stop_23); // stop module 23
                done = setpoint_.publish(); // publish new message to stop module 23 -> Lesser torque
             }   
        }
       else {
            ROS_ERROR("Failed to call service joint state");
        } 
   }
   return done;
}
