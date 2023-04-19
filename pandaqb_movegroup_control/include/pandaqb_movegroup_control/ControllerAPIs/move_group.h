#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

namespace rvt = rviz_visual_tools;

class MoveGroup{
public:
    MoveGroup();
    ~MoveGroup();

    void moveToHomePose();

    geometry_msgs::Pose getCurrentPose();
    geometry_msgs::Pose::_orientation_type getCurrentOrientation();

    void printCurrentJointPosition();

    void cartesianSpaceMotion(std::vector<geometry_msgs::Pose>& waypoints);
    void moveArmToPose(const geometry_msgs::Pose& pose);

protected:
    void moveJointSpace(std::vector<double>);
    
    // Moveit visualization attributes
    std::shared_ptr<moveit_visual_tools::MoveItVisualTools> visual_tools;
    Eigen::Isometry3d text_pose;

private:
    // Function to plan and execute a plan
    void PlanExecute();

    // Used in the constructor
    void LoadParams();
    void InitVisualTools();

    // Function to visualize the planed path
    void PlanVisualize(geometry_msgs::Pose target_pose);

    std::string PLANNING_GROUP;

    // Move Group attributes
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    
    // Ros param attributes
    std::vector<double> home;
    geometry_msgs::Pose home_pose;
    double planning_time, planning_attempts, velocity_scaling, acceleration_scaling;
};