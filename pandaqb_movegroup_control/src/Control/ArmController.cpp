#include <pandaqb_movegroup_control/Control/ArmController.h>

ArmController::ArmController(): arm_mover("arm"){
    std::string eef_frame, rbt_conf;
    ros::param::get("robot_config", rbt_conf);

    if(rbt_conf == "qb_hand"){
        eef_frame = "qbhand2m1_end_effector_link";
    }
    else if(rbt_conf == "gripper"){
        eef_frame = "panda_hand_tcp";
    }

    arm_mover.set_EEF_link(eef_frame);
    std::string path = "move_group/arm/";
    }

ArmController::~ArmController(){
}
void ArmController::approach_grasp(){
    ROS_INFO_STREAM("New plan: PRE GRASP POSE");

    geometry_msgs::Pose target_pose = arm_mover.getCurrentPose();
    target_pose.position.z -= 0.2;

    arm_mover.apend_waypt(target_pose);
    arm_mover.apend_waypt(pre_grasp_pose);

    arm_mover.build_cart_plan();
    arm_mover.clear_waypt();
}

void ArmController::move_to_g_pose(){
    ROS_INFO_STREAM("New plan: GRASP POSE");

    arm_mover.moveTo(grasp_pose);
}

void ArmController::set_grasp(geometry_msgs::Pose pose){
    grasp_pose = pose;
    compute_pre_grasp_pose();
}

void ArmController::pick_up(){
    ROS_INFO_STREAM("New plan: PICK UP");

    arm_mover.moveTo(pre_grasp_pose);
    arm_mover.moveHome();
}
geometry_msgs::Pose ArmController::get_current_pose(){
    return arm_mover.getCurrentPose();
}

void ArmController::compute_pre_grasp_pose(){
    pre_grasp_pose = grasp_pose;

    compute_normal_offset(grasp_pose.orientation);
    pre_grasp_pose.position.x += offsets[0];
    pre_grasp_pose.position.y += offsets[1];
    pre_grasp_pose.position.z += offsets[2];
    
    std::cout << "The pre grasp pose is:\n" << pre_grasp_pose << std::endl;
}

void ArmController::compute_normal_offset(geometry_msgs::Quaternion orient){
    Eigen::Quaterniond quad = orient_msg_to_eigen(orient);

    Eigen::Vector3d normal_offsets(0, 0, -0.05);
    std::cout << "The offsets in target grasp coordinates are:\n"
        << normal_offsets << std::endl;

    offsets = quad.matrix()*normal_offsets;

    std::cout << "The offsets in abs coordinates are:\n"
        << offsets << std::endl;
}

Eigen::Quaterniond ArmController::orient_msg_to_eigen(geometry_msgs::Quaternion orient){
    Eigen::Quaterniond quad;
    quad.x() = orient.x;
    quad.y() = orient.y;
    quad.z() = orient.z;
    quad.w() = orient.w;
    return quad;
}