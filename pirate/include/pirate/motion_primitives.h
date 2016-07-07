#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <actionlib/server/simple_action_server.h>
#include <sensor_msgs/JointState.h>
#include <pirate/PirateAction.h>
#include <pirate_srv/joint_state.h>
#include <pirate_msgs/PirateRPY.h>
#include <pirate/setpoint.h>
#include <pirate/control.h>
#include <stdlib.h> // abs
#include <iostream>		//Input-Output Streams
#include <math.h>		//Math library for M_PI
#include <boost/thread.hpp>

#ifndef MOTION_PRIMITIVES_H_
#define MOTION_PRIMITIVES_H_

    #define Motor0 0
    #define Motor1 1
    #define BendModule27 27
    #define BendModule26 26
    #define BendModule23 23
    #define BendModule22 22
    #define RotationModule 25
    #define FrontModule 21
    #define CameraModule 20
    #define RearModule 28
    #define Drive 22
    
    #define error_setpoint 0

    #define Conv_Rad_Setpoint 2133.3333
    #define Setpoint_CR_26 -256
    #define Setpoint_CR_27 256
    #define Position_CR_27_110 -1.25
    #define Position_CR_26_110 0.65
    #define Position_CR_27_90 -0.85
    #define Position_CR_26_90 0.40
    #define Position_Stop_26 0
    #define Setpoint_UNC_27 -85
    #define Setpoint_UNC_26 130
    #define Position_UNCRP_27_110 -0.9
    #define Position_UNCRP_26_110 0.45
    #define Position_UNCRP_27_90 -0.5
    #define Position_UNCRP_26_90 0.25
    #define Position_UNCR_27 -0.02
    #define Position_UNCR_26 0.02
    
    #define Setpoint_CF_23 -256
    #define Setpoint_CF_22 256
    #define Position_CF_22_110 -1.25
    #define Position_CF_23_110 0.65
    #define Position_CF_22_90 -0.85
    #define Position_CF_23_90 0.40
    #define Position_Stop_23 0
    #define Setpoint_UNC_22 -85
    #define Setpoint_UNC_23 130
    #define Position_UNCRP_22_110 -0.9
    #define Position_UNCRP_23_110 0.45
    #define Position_UNCRP_22_90 -0.5
    #define Position_UNCRP_23_90 0.25
    #define Position_UNCR_22 -0.02
    #define Position_UNCR_23 0.02
    
    #define Max_Acceptable_Rot_Tol 0.1
    #define Min_Acceptable_Rot_Tol -0.1
    
    #define MM_per_Count 0.083
    #define Conv_to_20ms 0.02
    #define Max_velocity 46

class Motion_Primitives {
   
   
    
    public:
    // constructor
    Motion_Primitives(Setpoint& setpoint, Control& control);
    
    // destructor
    ~Motion_Primitives();
    
    ros::ServiceClient joint_client;
    pirate_srv::joint_state joint_srv;
    
    // Motion primitives methods
    void moveBFModule(uint8_t moduleID, int setpoint);
    void changeControlBFModule(uint8_t moduleID, uint8_t control_mode);
    void changeControlPanCam(uint8_t control_mode);
    void changeControlDrive(uint8_t control_mode);
    void driveVel(int velocity);
    bool clampRear110(void);
    void rotateRad(double radians);
    bool clampRear90(void);
    bool unclampPartialRear110(void);
    bool unclampPartialRear90(void);
    bool unclampFullRear(void);
    
    bool clampFront110(void);
    bool clampFront90(void);
    bool unclampPartialFront110(void);
    bool unclampPartialFront90(void);
    bool unclampFullFront(void);
    
    private:
    ros::NodeHandle nh_;
    Setpoint& setpoint_;
    Control& control_;
};
#endif
