#include <pandaqb_movegroup_control/MoveGroup/GroupMover.h>

class ArmController{
public:
    ArmController();
    ~ArmController();

    void set_grasp_pose(geometry_msgs::Pose grasp);
    void approach_grasp();
    void pick_up();
    geometry_msgs::Pose get_current_pose();

private:
    void compute_pre_grasp_pose();
    void compute_normal_offset(geometry_msgs::Quaternion orient);

    GroupMover arm_mover;
    geometry_msgs::Pose grasp_pose, pre_grasp_pose;
    Eigen::Vector3d offsets;
};