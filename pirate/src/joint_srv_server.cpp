#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <pirate_srv/joint_state.h>
#include <boost/thread.hpp>


// global variable
sensor_msgs::JointState joint;
boost::mutex joint_mutex;

bool send_joint(pirate_srv::joint_state::Request  &req,
         pirate_srv::joint_state::Response &res)
{
  joint_mutex.lock();
  res.joint_state=joint;
  joint_mutex.unlock();
  return true;
}

void jointCB(const sensor_msgs::JointState::ConstPtr& state)
{
        
        joint_mutex.lock();

        /***************************
        / Update joint_state 
	****************************/    
	joint.header.stamp = state->header.stamp;
        joint.name.resize(15);
        joint.position.resize(15);
        joint.effort.resize(15);

	//front module
        joint.name[0] = "jointbend_base_link_mod1";
        joint.position[0] = state->position[0];
        joint.effort[0] = state->effort[0];
       

	//bending and rotational joints:
        joint.name[1] = "jointbend_mod1_mod2";
        joint.position[1] = state->position[1];
        joint.effort[1] = state->effort[1];
        joint.name[2] = "jointbend_mod2_mod3";
        joint.position[2] = state->position[2];
        joint.effort[2] = state->effort[2];
        joint.name[3] = "jointrot_mod3_mod4";
        joint.position[3] = state->position[3];
	joint.effort[3] = state->effort[3]; // No torque measured for rotation
        joint.name[4] = "jointbend_mod4_mod5";
        joint.position[4] =state->position[4];
        joint.effort[4] = state->effort[4];
        joint.name[5] = "jointbend_mod5_mod6";
        joint.position[5] = state->position[5];
        joint.effort[5] = state->effort[5];

	//rear module
        joint.name[6] = "jointbend_mod6_rearmod";
        joint.position[6] = state->position[6];
        joint.effort[6] = state->effort[6];

        //pan tilt camera
        joint.name[7] = "jointbend_base_link_camTiltBase";
        joint.position[7] = state->position[7];
        joint.effort[7] = state->effort[7];
        joint.name[8] = "jointbend_camTiltBase_camPanBase";
        joint.position[8] = state->position[8];
        joint.effort[8] = state->effort[8];

	//Wheels
        joint.name[9] = "jointrev_mod1_wheel";
        joint.position[9] =  state->position[9];
        joint.name[10] = "jointrev_mod2_wheel";
        joint.position[10] =  state->position[10];
        joint.name[11] = "jointrev_mod3_wheel";
        joint.position[11] = state->position[11];
        joint.name[12] = "jointrev_mod4_wheel";
        joint.position[12] =  state->position[12];
        joint.name[13] = "jointrev_mod5_wheel";
        joint.position[13] = state->position[13];
        joint.name[14] = "jointrev_mod6_wheel";
        joint.position[14] =  state->position[14]; 
      
        joint_mutex.unlock();
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "joint_state_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("jointstate", send_joint);
  ros::Subscriber sub_joint = n.subscribe("joint_states", 1, &jointCB);
  ROS_INFO("Ready to send joint state");
  ros::spin();

  return 0;
}
