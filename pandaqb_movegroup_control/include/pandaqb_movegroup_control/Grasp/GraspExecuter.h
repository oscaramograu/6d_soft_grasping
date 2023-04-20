#include <pandaqb_movegroup_control/MoveGroup/GroupMover.h>
#include <pandaqb_movegroup_control/Grasp/Grasp.h>

class GraspExecuter{
public:
    GraspExecuter();
    ~GraspExecuter();

    void approach(Grasp grasp);

    void grasp(Grasp TargetGrasp);

private:
    geometry_msgs::Pose graspToPose(Grasp TargetGrasp);
    void graspToOrientation();

    GroupMover Hand, Arm;
    std::vector<double> closed_hand, opened_hand;
};