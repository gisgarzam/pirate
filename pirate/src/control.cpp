#include <pirate/control.h>


// constructor
Control::Control(void){
ROS_INFO("Control class constructed");
pub_control=nh_.advertise<pirate_msgs::PirateControl_Array>("/pirate/control_array", 1);
}

//destructor
Control::~Control(void)
{    
}
/* Function that changes the controlMode of the array indicated by the moduleID and Motor, as referenced in Arduino MEGA.
Result -> done= true when everything went ok
          done= false when there was an error (not defined combination).
*/
void Control::setElement(uint8_t moduleID, uint8_t Motor, uint8_t controlMode){
  //  bool done= false;
    // if motor is 0 then they will be mapped in the first 0-8 positions
    if (Motor==0){
        switch(moduleID){
            case 20: // Tilt Camera
                control_msg.controlMode[0]= controlMode;
              //  done= true;
            break;
            case 21: // Front Module
                control_msg.controlMode[1]= controlMode;
              //  done= true;
            break;
            case 22: // Bend module 22
                control_msg.controlMode[2]= controlMode;
             //   done= true;
            break;
            case 23: // Bend module 23
                control_msg.controlMode[3]= controlMode;
              //  done= true;
            break;
            case 24: // IMU Module (no control mode to be changed)
            break;
            case 25: // Rotation module (no control mode to be changed)
         
            break;
            case 26: // Bend module 26
                control_msg.controlMode[6]= controlMode;
              //  done= true;
            break;
            case 27: // Bend module 27
                control_msg.controlMode[7]= controlMode;
               // done= true;
            break;
            case 28: // Module 28 (setpoint is sent but does not have effect)
                control_msg.controlMode[8]= controlMode; // This module is currently not controlled but implementation would be similar.
               // done= true;
            break;
            default: // Combination doesnt exist
            break;
        }
    }
    else if (Motor== 1) {
        switch(moduleID){
            case 20:  // Pan Camera
                control_msg.controlMode[9]= controlMode;
             //   done= true;
            break;
            case 21:  // Combination not possible
            break;
            case 22: // Wheels (Drive)
                control_msg.controlMode[11]= controlMode;
            //    done= true;
            break;
            case 23: // Combination doesnt exist
            break;
            case 24: // Combination doesnt exist
            break;
            case 25: // Combination doesnt exist
            break;
            case 26: // Combination doesnt exist
            break;
            case 27: // Combination doesnt exist   
            break;
            case 28: // Combination doesnt exist
            break;
            default: // Combination doesnt exist
            break;
        }
    }
   // return done;
}

// Change all the elements of the control array message at once
void Control::setArray(uint8_t control_array[12]) {
 //   bool done;
    for (int i=0; i <12; i++){
        control_msg.controlMode[i]=control_array[i];
    }
  //  done = true;
  //  return done;
}

// get an element value of the setpoint array message
int16_t Control::getElement(uint8_t moduleID, uint8_t Motor){
      int16_t element= ErrorValue; // has the maximum min value by default
    // if motor is 0 then they will be mapped in the first 0-8 positions
    if (Motor==0){
        switch(moduleID){
            case 20: // Tilt Camera
                element= control_msg.controlMode[0];
            break;
            case 21: // Front Module
                element= control_msg.controlMode[1];
            break;
            case 22: // Bend module 22
                element= control_msg.controlMode[2];
            break;
            case 23: // Bend module 23
                element= control_msg.controlMode[3];
            break;
            case 24: // IMU Module (no controlMode to be accessed)
            break;
            case 25: // Rotation module (no controlMode to be accessed)
            break;
            case 26: // Bend module 26
                element= control_msg.controlMode[6];
            break;
            case 27: // Bend module 27
                element= control_msg.controlMode[7];
            break;
            case 28: // Module 28 (controlMode is sent but does not have effect)
                element= control_msg.controlMode[8];
            break;
            default: // Combination doesnt exist
            break;
        }
    }
    else if (Motor== 1) {
        switch(moduleID){
            case 20:  // Pan Camera
                element= control_msg.controlMode[9];
            break;
            case 21:  //Combination doesnt exist;
            break;
            case 22: // Wheels (Drive)
                element= control_msg.controlMode[11];
            break;
            case 23: // Combination doesnt exist
            break;
            case 24: // Combination doesnt exist
            break;
            case 25: // Combination doesnt exist
            break;
            case 26: // Combination doesnt exist
            break;
            case 27: // Combination doesnt exist   
            break;
            case 28: // Combination doesnt exist
            break;
            default: // Combination doesnt exist
            break;
        }
    }
    return element;
}

// Publish the current setpoint array message
bool Control::publish(void){
    bool done;
    pub_control.publish(control_msg);
    done = true;
    return done;
}

