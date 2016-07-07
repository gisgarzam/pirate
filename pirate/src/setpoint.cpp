#include <pirate/setpoint.h>


// constructor
Setpoint::Setpoint(void){
ROS_INFO("Setpoint class constructed");
pub_setpoint=nh_.advertise<pirate_msgs::PirateSetpoint_Array>("/pirate/setpoint_array", 1);
}

//destructor
Setpoint::~Setpoint(void)
{    
}
/* Function that changes the setpoint of the array indicated by the moduleID and Motor, as referenced in Arduino MEGA.
Result -> done= true when everything went ok
          done= false when there was an error.
*/
void Setpoint::setElement(uint8_t moduleID, uint8_t Motor, int16_t setpoint){
 //   bool done= false;
    // if motor is 0 then they will be mapped in the first 0-8 positions
    if (Motor==0){
        switch(moduleID){
            case 20: // Tilt Camera
                setpoint_msg.setpoint[0]= setpoint;
          //      done= true;
            break;
            case 21: // Front Module
                setpoint_msg.setpoint[1]= setpoint;
          //      done= true;
            break;
            case 22: // Bend module 22
                setpoint_msg.setpoint[2]= setpoint;
          //      done= true;
            break;
            case 23: // Bend module 23
                setpoint_msg.setpoint[3]= setpoint;
          //      done= true;
            break;
            case 24: // IMU Module (no setpoint should be changed)
            break;
            case 25: // Rotation module
                setpoint_msg.setpoint[5]= setpoint;
          //      done= true;
            break;
            case 26: // Bend module 26
                setpoint_msg.setpoint[6]= setpoint;
          //     done= true;
            break;
            case 27: // Bend module 27
                setpoint_msg.setpoint[7]= setpoint;
           //     done= true;
            break;
            case 28: // Module 28 (setpoint is sent but does not have effect)
                setpoint_msg.setpoint[8]= setpoint; // This module is currently not controlled but implementation would be similar.
          //      done= true;
            break;
            default: // Combination doesnt exist
            break;
        }
    }
    else if (Motor== 1) {
        switch(moduleID){
            case 20:  // Pan Camera
                setpoint_msg.setpoint[9]= setpoint;
           //     done= true;
            break;
            case 21:  // Front LED
                setpoint_msg.setpoint[10]= setpoint;
           //     done= true;
            break;
            case 22: // Wheels (Drive)
                setpoint_msg.setpoint[11]= setpoint;
          //      done= true;
            break;
            case 23: // Combination doesnt exist
            break;
            case 24: // Combination doesnt exist
            break;
            case 25: // Combination doesnt exist
            break;
            case 26: // Combination doesnt exist
            break;
            case 27: // Combination doesnt exist   
            break;
            case 28: // Rear LED
                setpoint_msg.setpoint[12]= setpoint;
           //     done=true;
            break;
            default: // Combination doesnt exist
            break;
        }
    }
  //  return done;
}

// Change all the elements of the setpoint array message at once
void Setpoint::setArray(int16_t setpoint_array[13]) {
  //  bool done;
    for (int i=0; i <13; i++){
        setpoint_msg.setpoint[i]=setpoint_array[i];
    }
 //   done = true;
 //   return done;
}

// get an element value of the setpoint array message
int16_t Setpoint::getElement(uint8_t moduleID, uint8_t Motor){
      int16_t element= ErrorValue; // has the maximum min value by default
    // if motor is 0 then they will be mapped in the first 0-8 positions
    if (Motor==0){
        switch(moduleID){
            case 20: // Tilt Camera
                element= setpoint_msg.setpoint[0];
            break;
            case 21: // Front Module
                element= setpoint_msg.setpoint[1];
            break;
            case 22: // Bend module 22
                element= setpoint_msg.setpoint[2];
            break;
            case 23: // Bend module 23
                element= setpoint_msg.setpoint[3];
            break;
            case 24: // IMU Module (no setpoint to be accessed)
            break;
            case 25: // Rotation module
                element= setpoint_msg.setpoint[5];
            break;
            case 26: // Bend module 26
                element= setpoint_msg.setpoint[6];
            break;
            case 27: // Bend module 27
                element= setpoint_msg.setpoint[7];
            break;
            case 28: // Module 28 (setpoint is sent but does not have effect)
                element= setpoint_msg.setpoint[8];
            break;
            default: // Combination doesnt exist
            break;
        }
    }
    else if (Motor== 1) {
        switch(moduleID){
            case 20:  // Pan Camera
                element= setpoint_msg.setpoint[9];
            break;
            case 21:  // Front LED
                element= setpoint_msg.setpoint[10];
            break;
            case 22: // Wheels (Drive)
                element= setpoint_msg.setpoint[11];
            break;
            case 23: // Combination doesnt exist
            break;
            case 24: // Combination doesnt exist
            break;
            case 25: // Combination doesnt exist
            break;
            case 26: // Combination doesnt exist
            break;
            case 27: // Combination doesnt exist   
            break;
            case 28: // Rear LED
                element= setpoint_msg.setpoint[12];
            break;
            default: // Combination doesnt exist
            break;
        }
    }
    return element;
}

// Publish the current setpoint array message
bool Setpoint::publish(void){
    bool done;
    pub_setpoint.publish(setpoint_msg);
    done = true;
    return done;
}

