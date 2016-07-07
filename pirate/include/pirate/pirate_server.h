#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <actionlib/server/simple_action_server.h>
#include <sensor_msgs/JointState.h>
#include <pirate/PirateAction.h>
#include <pirate_srv/server_status.h>
#include <pirate_msgs/PirateControl_Array.h>
#include <pirate_msgs/PirateLimits_Array.h>
#include <pirate_msgs/PirateSetpoint_Array.h>
#include <pirate_msgs/PirateRPY.h>
#include <pirate/clamp_straight.h>
#include <pirate/motion_primitives.h>
#include <pirate/setpoint.h>
#include <pirate/control.h>
#include <iostream>		//Input-Output Streams
#include <math.h>		//Math library for M_PI
#include <boost/thread.hpp>

#ifndef PIRATE_SERVER_H_
#define PIRATE_SERVER_H_


typedef actionlib::SimpleActionServer<pirate::PirateAction> PirateActionServer;
//typedefs to help us out with the action server so that we don't hace to type so much
//typedef actionlib::SimpleActionServer<pirate::PirateAction> PirateActionServer;

  enum PirateState {
    IDLE_UNCLAMPED,
    IDLE_CLAMPED,
    CLAMPING_STRAIGHT,
    CLAMPING_SIDEWAYS,
    TAKING_TJOINT,
    TAKING_MITRE_BEND,
    HOMING,
    DRIVING
  };
  
    enum TjointState {
        TJOINT_CHANGING_MODE,
        TJOINT_MOVING,
        TJOINT_CLAMPING_REAR,
        TJOINT_CLAMPING_FRONT,
        TJOINT_UNCLAMPING_REAR,
        TJOINT_UNCLAMPING_FRONT,
        TJOINT_ROTATING_REAR,
        TJOINT_ROTATING_FRONT,
        TJOINT_DRIVING
    };
    
    enum Server_State{
      ACTIVE,
      PREEMPTED,
      IDLE
    };
  

// A class that uses the actionlib::ActionServer interface that moves the robot according to the different actions (clamp, take joints, etc).
class Pirate_Server {
    
    
    
    public:
    // constructor
    Pirate_Server(Setpoint& setpoint, Control& control);
     
    // destructor
    ~Pirate_Server();
    
    
    private:
    Setpoint& setpoint_;
    Control& control_;
    // Class for motion primitives
    Motion_Primitives motion_primitives; 
    //ros::NodeHandle nh_;
   // ros::NodeHandle nh_;
    std::string action_name_;
    // ** CALLBACK FUNCTIONS ** //
    // callback function that will execute whenever there is a goal available
    void executeCB(const pirate::PirateGoalConstPtr &goal);
    // callback function that will execute if the action is preempted
    void preemptCB(); 
    // service function to send status
    bool send_status(pirate_srv::server_status::Request  &req,
         pirate_srv::server_status::Response &res);
    // callback function for subscriber of the joint states
    void jointCB(const sensor_msgs::JointState::ConstPtr& state);
    // callback function for subscriber of the imu roll, pitch and yaw data
    void imuCB(const pirate_msgs::PirateRPY::ConstPtr& imu_state);
    
    //Pointer to PirateActionServer object
    PirateActionServer* as_;
    
    
    
    // variables used in different libraries
    PirateState state_;
    Server_State server_state;
    
    
    // Sub-classes for actions
    Clamp_Straight* clamp_straight; // pointer to a clamp_straight object
    
    // Class for motion primitives
    //Motion_Primitives motion_primitives; 
    
    
    // ** HELPER MINI- FUNCTIONS ** //
    // function to get the difference in radians between the current roll angle and the desired angle.
    //double get_difference_rad(double roll, double desired);
    
    // ** PARAMETER CONTAINERS ** //
    //int16_t setp_clamp_up_90_step1[18], setp_clamp_up_90_step2[18], setp_clamp_up_90_step3[18], setp_clamp_up_90_step4[18], setp_clamp_up_90_step5[18], setp_clamp_up_90_step6[18], setp_clamp_up_90_step7[18], setp_clamp_up_90_step8[18],setp_clamp_up_90_step9[18], setp_clamp_up_90_step10[18];
    //uint8_t cont_clamp_up_90_step1[18], cont_clamp_up_90_step2[18], cont_clamp_up_90_step3[18], cont_clamp_up_90_step4[18];
    
    // ** VARIABLES **/
    
    ros::ServiceServer Server_status;
    
    // create messages that are used to published feedback/result
    pirate::PirateFeedback feedback_;
    pirate::PirateResult result_;
    // subscriber for feedback of the joint states
    ros::Subscriber sub_joint;
    // subscriber for the feedback of the Roll, Pitch, Yaw orientation of the robot
    ros::Subscriber sub_rpy;
    // helper variables to contain the goals parameters and the success variable
    //char action, param1, param2;
    bool success;
    uint8_t action, pipeDiam;
    int16_t driveDist;
    // publishers for the setpoints and control modes and their messages
    ros::Publisher pub_setpoint;
    ros::Publisher pub_control;
    pirate_msgs::PirateSetpoint_Array setpoint_msg;
    pirate_msgs::PirateControl_Array control_msg;
    // joint state container
    sensor_msgs::JointState joint;
    //imu custom message container for roll, pitch and yaw
    pirate_msgs::PirateRPY imu;
    //mutex for threads
    boost::mutex status_mutex;
    // define upright position in degrees and slope
    int upright = 71;
    double m= 2133.33333; // this is the slope for the equation of the line that represent the digital setpoint (y) and the rotation of the pirate in radians (x)  
  };

#endif
