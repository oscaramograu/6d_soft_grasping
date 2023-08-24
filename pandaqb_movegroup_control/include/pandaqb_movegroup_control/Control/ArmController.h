#include <pandaqb_movegroup_control/MoveGroup/GroupMover.h>

class ArmController{
public:
    ArmController();
    ~ArmController();

    void set_grasp(geometry_msgs::Pose pose);
    void approach_grasp();
    void move_to_g_pose();
    void approach_place();
    void pick_up();
    geometry_msgs::Pose get_current_pose();

private:
    void compute_pre_grasp_pose();
    void compute_normal_offset(geometry_msgs::Quaternion orient);
    void move_to_pre_pose();
    bool check_right_grasp();

    Eigen::Quaterniond orient_msg_to_eigen(geometry_msgs::Quaternion orient);
    Eigen::Vector3d pose_msg_to_eigen(geometry_msgs::Point position);
    Eigen::Vector3d projectVectorOntoPlane(Eigen::Vector3d v);

    GroupMover arm_mover;
    geometry_msgs::Pose grasp_pose, pre_grasp_pose;
    Eigen::Vector3d offsets;

    std::vector<double> left_pose, right_pose;
    bool right_gr_flag;
};