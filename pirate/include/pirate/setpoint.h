#include <ros/ros.h>
#include <std_msgs/String.h>
#include <pirate_msgs/PirateSetpoint_Array.h>
#include <iostream>

#ifndef SETPOINT_H_
#define SETPOINT_H_

// This class contains the instance of a setpoint (array) message that will be used throughout the implementation of the Pirate Server Main Class //

/*MAPPING OF THE SETPOINT_ARRAY Message is given by:
          
            Element number:  0       1     2       3       4       5      6      7      8       9     10    11      12
 setpoint_msg.setpoint = [   X       X     X       X       X       X      X      X      X       X     X     X       X ]
                            20      21     22     23      24      25      26     27     28      20    21    22      28
                          -----------------------------------------------------------------  --------------------------
                                                   MOTOR 0                                            MOTOR 1
                                        (Bend [22,23,26,27]/rear[28]/imu[24]/)                 (20=Pan Camera, 
                                        front[21]/rotation[25]/camera[20] modules)              21=Front LED, 
                                                                                                22=Wheel drive, 
                                                                                                28=Rear LED)
 */   
 
      
class Setpoint {
    
    #define ErrorValue -32768;
    public:
    // empty constructor
    Setpoint();
    
    // destructor
    ~Setpoint();
    
    // public methods
    // set an element of the setpoint message which is an array of maximum 13 elements
    void setElement(uint8_t moduleID, uint8_t Motor, int16_t setpoint);
    void setArray(int16_t setpoint_array[13]);
    // Get methods are not actively used so only implemented in case one element needs to be retrieved
    int16_t getElement(uint8_t moduleID, uint8_t Motor);
    bool publish(void);
    
    private:
    ros::NodeHandle nh_;
    ros::Publisher pub_setpoint;
    pirate_msgs::PirateSetpoint_Array setpoint_msg;
};
#endif