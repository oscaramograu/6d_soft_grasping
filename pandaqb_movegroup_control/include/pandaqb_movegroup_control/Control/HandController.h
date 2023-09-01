#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

class HandController{
public:
    HandController(ros::NodeHandle *nh);
    ~HandController();
    
    void grasp(std::vector<double> synergies);

    void open();
    
    void print_joints();

private:
    void publish_traj(std::vector<double> synergies);
    void init_point();

    ros::Publisher traj_syn_pub;
    trajectory_msgs::JointTrajectory traj_msg;
    trajectory_msgs::JointTrajectoryPoint point;
};    