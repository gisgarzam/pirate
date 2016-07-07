
#include <pirate/pirate_server.h>
#include <iostream>		//Input-Output Streams
#include <math.h>		//Math library for M_PI
#include <pirate_srv/server_status.h>
#include <boost/thread.hpp>


  
Pirate_Server::Pirate_Server(Setpoint& setpoint, Control& control) :
setpoint_(setpoint),
control_(control),
motion_primitives(setpoint_,control_)
{ 
    as_= new PirateActionServer (ros::NodeHandle(), "pirate", boost::bind(&Pirate_Server::executeCB, this, _1), false);
    ROS_INFO("Action Server is being constructed");
    // Constructing action server
    as_->registerPreemptCallback(boost::bind(&Pirate_Server::preemptCB, this));
    ros::NodeHandle nh_;
    Server_status = nh_.advertiseService("pirate_server_status", &Pirate_Server::send_status, this);
    //pub_control = nh_.advertise<pirate_msgs::PirateControl_Array>("/pirate/control_array", 1);
    //pub_setpoint = nh_.advertise<pirate_msgs::PirateSetpoint_Array>("/pirate/setpoint_array", 1);
    as_->start();
    ROS_INFO("Action Server is now running");
    //motion_primitives = new Motion_Primitives(setpoint_, control_);
    ROS_INFO("SERVER: Creating Motion Primitives object");
    clamp_straight = new Clamp_Straight(setpoint_, control_, motion_primitives); // create the instance of clamp_staright and send the references to the object Setpoint and Control
    ROS_INFO("SERVER: Creating actions Clamp_Straight object");
    state_ = IDLE_UNCLAMPED;
  }

  Pirate_Server::~Pirate_Server(void)
  {
  } 
  
  /*bool send_status(pirate_srv::server_status::Request  &req,
         pirate_srv::server_status::Response &res)
  {
  status_mutex.lock();
  res.state=pirate_srv::server_status::state;
  joint_mutex.unlock();
  return true;
}*/

// Function for service to send status... returns true if the goal is active, returns false otherwise.
  bool Pirate_Server::send_status(pirate_srv::server_status::Request  &req, pirate_srv::server_status::Response &res){
    if (as_->isActive()){
      res.state= true; 
    }
    else{
      res.state=false;
    }
    return true;
  }
  
   
  void Pirate_Server::executeCB(const pirate::PirateGoalConstPtr &goal)
  {
    //server_state= ACTIVE;
    success=true;
    action = goal->name;
    pipeDiam = goal->pipeDiameter;
    driveDist = goal->distance;
    // publish info to the console for the user
    ROS_INFO("Pirate: Executing, creating sequence for pirate with %d mm diameter", pipeDiam);
    
    // start executing the action
    switch (action){
   
	        case pirate::PirateGoal::CLAMP_STRAIGHT:
                state_= CLAMPING_STRAIGHT; // Change state while it is performing the action
                ROS_INFO("Clamping Straight");
                // ROS_INFO_STREAM("executeCB: [thread=" << boost::this_thread::get_id() << "]");
                success= clamp_straight->execute(pipeDiam); // will return with success true or false.
                std::cout << "Pirate has succeeded? " << success << std::endl;
                state_=IDLE_CLAMPED; // return to IDLE but now we know we are clamped after completing function.
           break;
      
           case pirate::PirateGoal::CLAMP_SIDEWAYS:
                state_= CLAMPING_SIDEWAYS;
                ROS_INFO("Clamping Sideways");
                //success=actions::take_t_joint(param1, param2); // will return with success true or false.
                state_=IDLE_CLAMPED; // return to IDLE but now we know we are clamped after completing function.
           break;
      
           default:
           break;
	}
    
    std::cout << "Value of success is " << success << std::endl;
    if(success)
	    {
	     // result_.sequence = feedback_.sequence;
	       ROS_INFO("Succeeded");
	       // set the action state to succeeded
                result_.joint_state= feedback_.joint_state;
	        as_->setSucceeded(result_);
  	    }
    // server_state= IDLE;
}

 void Pirate_Server::preemptCB()
  {
       //   server_state= PREEMPTED;
	  //ROS_INFO_STREAM("preemptCB: thread=" << boost::this_thread::get_id() << "]");
	  //notify the ActionServer that we've successfully preempted
          ROS_INFO("Pirate Preempted");
          // set success to false
 	        success = false;
         // ROS_DEBUG_NAMED("pirate","Move base preempting the current goal");
          as_->setPreempted();
        //  control_msg.controlMode[8]=8;// PWM=0, Vel=1, Pos=2, Torque=3, Curr=4, CALIB= 5, TorqueLimit=6, PLAY=7, STOP=8, noLimitsAll=9
	//  pub_control.publish(control_msg);
          //we'll actually return from execute after preempting
          return;
}

