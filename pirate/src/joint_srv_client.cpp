#include <ros/ros.h>
#include <pirate_srv/joint_state.h>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "joint_state_client");

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<pirate_srv::joint_state>("jointstate");
  pirate_srv::joint_state joint_srv;
  

 if (client.call(joint_srv))
  {
     ROS_INFO_STREAM( "Received joint. For example joint[0] position is " << joint_srv.response.joint_state.position[0]);
  }
  else
  {
    ROS_ERROR("Failed to call service jointstate");
    return 1;
  }

  return 0;
}
