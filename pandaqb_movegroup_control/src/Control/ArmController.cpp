#include <pandaqb_movegroup_control/Control/ArmController.h>

ArmController::ArmController(): arm_mover("arm"){
    arm_mover.set_EEF_link("qbhand2m1_end_effector_link");
}

ArmController::~ArmController(){
}
void ArmController::approach_grasp(){
    // arm_mover.moveHome(); 

    ROS_INFO_STREAM("Approaching to pre grasp pose");
    arm_mover.moveTo(pre_grasp_pose);

    ROS_INFO_STREAM("Approaching to grasp pose");
    arm_mover.moveTo(grasp_pose);
}

void ArmController::set_grasp_pose(geometry_msgs::Pose grasp){
    grasp_pose = grasp;
    compute_pre_grasp_pose();
}

void ArmController::pick_up(){
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
    Eigen::Quaterniond quad;
    quad.x() = orient.x;
    quad.y() = orient.y;
    quad.z() = orient.z;
    quad.w() = orient.w;

    Eigen::Vector3d normal_offsets(0, 0, -0.05);
    std::cout << "The offsets in target grasp coordinates are:\n"
        << normal_offsets << std::endl;

    offsets = quad.matrix().inverse()*normal_offsets;

    std::cout << "The offsets in abs coordinates are:\n"
        << offsets << std::endl;
}

