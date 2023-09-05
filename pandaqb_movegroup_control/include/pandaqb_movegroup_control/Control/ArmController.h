#include <pandaqb_movegroup_control/MoveGroup/GroupMover.h>

class ArmController{
public:
    ArmController();
    ~ArmController();

    void set_grasp(geometry_msgs::Pose pose);
    void set_place_pose(geometry_msgs::Pose pose);
    void set_tcp(std::string tcp);

    void approach_grasp();
    void move_to_g_pose();
    void move_to_place_pose();
    void pick_up();
    void move_home();

    geometry_msgs::Pose get_current_pose();

private:
    geometry_msgs::Pose compute_pre_pose(geometry_msgs::Pose final_pose);

    void compute_place_pose();
    Eigen::Vector3d compute_normal_offset(geometry_msgs::Quaternion orient);
    Eigen::Quaterniond orient_msg_to_eigen(geometry_msgs::Quaternion orient);

    GroupMover arm_mover;
    geometry_msgs::Pose grasp_pose, place_pose, pre_grasp_pose, pre_place_pose;
    double default_v, default_a;
};