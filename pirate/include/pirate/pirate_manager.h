#include <ros/ros.h>
#include <pirate/setpoint.h>
#include <pirate/control.h>
#include <pirate/motion_primitives.h>
#include <pirate/pirate_server.h>
#include <iostream>		//Input-Output Streams
#include <math.h>		//Math library for M_PI
#include <boost/thread.hpp>


#ifndef PIRATE_MANAGER_H_
#define PIRATE_MANAGER_H_

class Pirate_Manager{
    public:
    // constructor
    Pirate_Manager(Setpoint& setpoint, Control& control);
    
    //destructor
    ~Pirate_Manager();

    
    private:
   
    
    Setpoint& setpoint_;
    Control& control_;
    Pirate_Server* Server; // pointer to a server of the ActionLib implemented for the Pirate
 
};
#endif