#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <actionlib/server/simple_action_server.h>
#include <sensor_msgs/JointState.h>
#include <pirate/PirateAction.h>
#include <pirate_msgs/PirateControl_Array.h>
#include <pirate_msgs/PirateLimits_Array.h>
#include <pirate_msgs/PirateSetpoint_Array.h>
#include <pirate_srv/server_status.h>
#include <pirate_srv/joint_state.h>
#include <pirate_srv/imu.h>
#include <pirate/setpoint.h>
#include <pirate/control.h>
#include <pirate_msgs/PirateRPY.h>
#include <stdlib.h> // abs
#include <iostream>		//Input-Output Streams
#include <math.h>		//Math library for M_PI
#include <boost/thread.hpp>
#include <pirate/motion_primitives.h>

enum ClampStraightState {
        CLAMP_ST_CHANGING_MODE,
        CLAMP_ST_MOVING,
        CLAMP_ST_CLAMPING_REAR,
        CLAMP_ST_CLAMPING_FRONT,
        CLAMP_ST_UNCLAMPING_REAR,
        CLAMP_ST_UNCLAMPING_FRONT,
        CLAMP_ST_ROTATING_REAR,
        CLAMP_ST_ROTATING_FRONT
    };

class Clamp_Straight {
    public:
    // constructor
    Clamp_Straight(Setpoint& setpoint, Control& control, Motion_Primitives& motion_primitives);
    
    // destructor
    ~Clamp_Straight();
    
    bool execute(uint8_t pipe_diameter);
    
    private:
    ros::NodeHandle n;
    Setpoint& setpoint_;
    Control& control_;
    Motion_Primitives& motion_primitives_;
    ros::ServiceClient joint_client;
    ros::ServiceClient status_client;
    ros::ServiceClient imu_client;
    pirate_srv::joint_state joint_srv;
    pirate_srv::imu imu_srv;
    pirate_srv::server_status status_srv;
    ros::Publisher pub_control;
    ros::Publisher pub_setpoint;
};