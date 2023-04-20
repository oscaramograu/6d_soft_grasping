#include <pandaqb_movegroup_control/MoveGroup/InfoTriggerMG.h>


class GroupMover : public InfoTriggerMoveGroup{
public:
    GroupMover(std::string planning_group);
    virtual ~GroupMover();

    void moveTo(const geometry_msgs::Pose& pose); // Cartesianspace
    void moveTo(std::vector<double> joints); // Jointspace

    void moveHome();

private:
    void planExecute();
};