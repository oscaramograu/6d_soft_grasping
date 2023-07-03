#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>

class HandControllerBase {
public:
    HandControllerBase();
    ~HandControllerBase();

    /**
    * Sends the actuations on the motors using ROS to close 
    * or open the hand. 
    * 
    * @param pos1 value from 0 to 1, which represent the 
    * actuation on the first motor 
    * @param pos2 value from 0 to 1, which represent the 
    * actuation on the second motor  
    * @param duration time spent to execute the closure
    *  when actuating to the motors
    **/
    void sendTrajectory(double pos1, double pos2, double duration);

private:
    ros::NodeHandle nh_;
    ros::Publisher pub_;
};