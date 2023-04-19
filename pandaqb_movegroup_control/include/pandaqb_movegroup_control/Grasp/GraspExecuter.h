#include <pandaqb_movegroup_control/MoveGroup/GroupMover.h>
#include <pandaqb_movegroup_control/Grasp/Grasp.h>

class GraspExecuter{
public:
    GraspExecuter();
    ~GraspExecuter();

    void approach(Grasp grasp);

    void grasp();

private:
    void graspToPose();
    void graspToOrientation();

    GroupMover Arm, Hand;
};