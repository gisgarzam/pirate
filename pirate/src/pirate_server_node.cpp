
#include <pirate/pirate_server.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pirate_node");

  // ros::AsyncSpinner s(5);
  //s.start();
 
  //rate for main thread
  
  pirate_action_server::Pirate_Server Server;
  ros::spin();
  
  
  //ros::waitForShutdown();

  return 0;
}