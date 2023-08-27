#include <pandaqb_movegroup_control/MoveGroup/GroupMover.h>

class ArmController{
public:
    ArmController();
    ~ArmController();

    void set_grasp(geometry_msgs::Pose pose);
    void approach_grasp();
    void move_to_g_pose();
    void approach_place();
    void place();
    void pick_up();
    geometry_msgs::Pose get_current_pose();

private:
    void compute_pre_grasp_pose();
    void compute_place_pose();
    void compute_normal_offset(geometry_msgs::Quaternion orient);
    Eigen::Quaterniond orient_msg_to_eigen(geometry_msgs::Quaternion orient);

    GroupMover arm_mover;
    geometry_msgs::Pose grasp_pose, pre_grasp_pose, place_pose;
    Eigen::Vector3d offsets;
};