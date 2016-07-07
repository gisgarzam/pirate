#include <pirate/pirate_manager.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pirate_manager_node");
  // Create setpoint and control objects (to publish setpoint and control messages)
  Setpoint setpoint;
  Control control;
  
  Pirate_Manager pirateManager(setpoint, control);
  ros::spin();
  
  
  //ros::waitForShutdown();

  return 0;
}