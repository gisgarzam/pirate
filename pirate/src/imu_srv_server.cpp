#include <ros/ros.h>
#include <pirate_srv/imu.h>
#include <pirate_msgs/PirateRPY.h>

// global variable
pirate_msgs::PirateRPY imu_message;

bool send_imu(pirate_srv::imu::Request  &req,
         pirate_srv::imu::Response &res)
{
  res.imu=imu_message;
  return true;
}

void imuCB(const pirate_msgs::PirateRPY::ConstPtr& imu_state)
  {
     // Copy the new information
     imu_message.roll= imu_state->roll;
     // cout << "Roll = " << imu_state->roll << endl;
     imu_message.pitch = imu_state->pitch;
     imu_message.yaw= imu_state->yaw;
   }


int main(int argc, char **argv)
{
  ros::init(argc, argv, "imu_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("imu_state", send_joint_state);
  ros::Subscriber sub_imu = n.subscribe("imu_rpy",1, &PirateAction::imuCB);
  ROS_INFO("Ready to send imu");
  ros::spin();

  return 0;
}
