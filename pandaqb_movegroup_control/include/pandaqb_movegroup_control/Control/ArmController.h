#include <pandaqb_movegroup_control/MoveGroup/GroupMover.h>

class ArmController{
public:
    ArmController();
    ~ArmController();

    void set_grasp_pose(geometry_msgs::Pose grasp);
    void approach_grasp();

private:
    void compute_pre_grasp_pose();
    void compute_normal_offset(geometry_msgs::Quaternion orient);


    GroupMover hand_mover;
    geometry_msgs::Pose grasp_pose, pre_grasp_pose;
    Eigen::Vector3d offsets;
};