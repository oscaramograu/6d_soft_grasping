#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/AttachedCollisionObject.h>

class BaseMoveGroup{
public:
    BaseMoveGroup(std::string planning_group);
    ~BaseMoveGroup();

protected: 
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    moveit::planning_interface::PlanningSceneInterface planning_scene_;
    moveit::planning_interface::MoveGroupInterface::Plan plan;

    std::string PLANNING_GROUP;
    std::vector<double> home;

private:
    /**
     * 
     **/
    void load_params();

    double planning_time, planning_attempts, velocity_scaling, acceleration_scaling;
};