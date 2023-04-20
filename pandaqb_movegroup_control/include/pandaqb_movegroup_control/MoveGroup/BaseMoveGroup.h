#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

class BaseMoveGroup{
public:
    BaseMoveGroup(std::string planning_group);
    ~BaseMoveGroup();

protected: 
    // Move Group attributes
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    moveit::planning_interface::MoveGroupInterface::Plan plan;

    std::string PLANNING_GROUP;
    std::vector<double> home;

private:
    void load_params();

    // Ros param attributes
    double planning_time, planning_attempts, velocity_scaling, acceleration_scaling;
};