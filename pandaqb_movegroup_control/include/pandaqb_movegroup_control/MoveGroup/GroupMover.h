#include <MoveGroup/InfoTriggerMG.h>


class GroupMover : InfoTriggerMoveGroup {
public:
    GroupMover();
    virtual ~GroupMover();

    moveit::planning_interface::MoveGroupInterface::Plan moveTo(const std::vector<geometry_msgs::Pose>& waypoints); // Cartesianspace
    moveit::planning_interface::MoveGroupInterface::Plan moveTo(const std::vector<double>& joints); // Jointspace

private:
    void planExecute();
};