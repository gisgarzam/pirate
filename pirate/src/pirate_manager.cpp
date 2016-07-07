#include <pirate/pirate_manager.h>

Pirate_Manager::Pirate_Manager(Setpoint& setpoint, Control& control):
setpoint_(setpoint),
control_(control)
{
    ROS_INFO("Creating Pirate Manager");
    Server= new Pirate_Server(setpoint_,control_);
}

Pirate_Manager::~Pirate_Manager(void) //empty constructor
{
}
