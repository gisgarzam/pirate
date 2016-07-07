#include <pirate/clamp_straight.h>

//Constructor
Clamp_Straight::Clamp_Straight(Setpoint& setpoint, Control& control, Motion_Primitives& motion_primitives):
setpoint_(setpoint),
control_(control),
motion_primitives_(motion_primitives)
 {
    pub_control = n.advertise<pirate_msgs::PirateControl_Array>("/pirate/control_array", 1);
    pub_setpoint = n.advertise<pirate_msgs::PirateSetpoint_Array>("/pirate/setpoint_array", 1);
    ROS_INFO("Creating Clamp Straight class instance");
    ROS_INFO("Service joint state, server status, and imu established.");
    joint_client = n.serviceClient<pirate_srv::joint_state>("jointstate");
    status_client = n.serviceClient<pirate_srv::server_status>("pirate_server_status");
    imu_client = n.serviceClient<pirate_srv::imu>("imu_state");
}

//Destructor
Clamp_Straight::~Clamp_Straight(void){
    // empty destructor
}


// This function can be used for bend modules and the front module ---> NOT MEANT TO BE USED WITH THE CAMERAS
// Possible modules are 21, 22, 23, 26 and 27
bool Clamp_Straight::execute(uint8_t pipe_diameter) {
    bool success= false;
    bool setpoint_published = false;
    bool control_published = false;
    for (int i=0; i<10; i++){
       if (status_client.call(status_srv)){
         if (status_srv.response.state)  {
             ROS_INFO("Pirate: Executing, creating sequence for pirate with %d mm diameter round %d", pipe_diameter,i);
             //setpoint_.setElement(22,0, i+32);
             motion_primitives_.moveBFModule(20+i, i+50);
             setpoint_published = setpoint_.publish();
             //control_.setElement(22,0, i+1);
             //control_published = control_.publish();
             
             ros::Duration(1).sleep();  
             success=true; 
         }
         else{
             success= false;
             ROS_INFO ("Cancelling clamp straight routine");
             break;
         }
       }
       else {
           ROS_ERROR("Could not find the server for service to get Pirate Server Status");
           success= false;
           break;
       }
    }
    return success;
}
