#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

class BaseMoveGroup{
public:
    BaseMoveGroup();
    ~BaseMoveGroup();

protected: 
    // Move Group attributes
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    moveit::planning_interface::MoveGroupInterface::Plan plan;


private:
    std::string PLANNING_GROUP;

    // Ros param attributes
    double planning_time, planning_attempts, velocity_scaling, acceleration_scaling;
};