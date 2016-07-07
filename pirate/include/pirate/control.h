#include <ros/ros.h>
#include <std_msgs/String.h>
#include <pirate_msgs/PirateControl_Array.h>
#include <iostream>

#ifndef CONTROL_H_
#define CONTROL_H_

// General macros
#define ErrorValue -32768;
// This class contains the instance of a setpoint (array) message that will be used throughout the implementation of the Pirate Server Main Class //

/*MAPPING OF THE CONTROL_ARRAY Message is given by:
          
            Element number:    0       1     2       3       4       5      6      7      8       9     10    11      
 control_msg.controlMode = [   X       X     X       X       X       X      X      X      X       X     X      X  ]
                              20      21     22     23      24      25      26     27     28      20    N/A    22 
                            -----------------------------------------------------------------  --------------------
                                                              MOTOR 0                               MOTOR 1
                                        (Bend [22,23,26,27]/rear[28]/imu[24] NOT USED/)           (20=Pan Camera, 
                                    front[21]/rotation[25] NOT USED/camera tilt[20] modules)      22=Wheel drive) 
                                                                                                
                                                                                             
 */   
 
      
class Control {

    public:
    // empty constructor
    Control();
    
    // destructor
    ~Control();
    
    // public methods
    // set an element of the setpoint message which is an array of maximum 13 elements
    void setElement(uint8_t moduleID, uint8_t Motor, uint8_t controlMode);
    void setArray(uint8_t control_array[12]);
    // Get methods are not actively used so only implemented in case one element needs to be retrieved
    int16_t getElement(uint8_t moduleID, uint8_t Motor);
    bool publish(void);
    
    private:
    ros::NodeHandle nh_;
    ros::Publisher pub_control;
    pirate_msgs::PirateControl_Array control_msg;
};

#endif