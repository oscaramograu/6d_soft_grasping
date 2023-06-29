#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>

class HandControllerBase {
public:
    HandControllerBase();
    ~HandControllerBase();

    void sendTrajectory(double pos1, double pos2, double duration); // seconds

private:
    ros::NodeHandle nh_;
    ros::Publisher pub_;
};