#include <pandaqb_movegroup_control/Control/ArmController.h>

ArmController::ArmController(): arm_mover("arm"){
    std::string eef_frame, rbt_conf;
    ros::param::get("robot_config", rbt_conf);

    if(rbt_conf == "qb_hand"){
        eef_frame = "power_link";
    }
    else if(rbt_conf == "gripper"){
        eef_frame = "panda_hand_tcp";
    }

    arm_mover.set_EEF_link(eef_frame);
    std::string path = "move_group/arm/";
}

ArmController::~ArmController(){
}

void ArmController::set_tcp(std::string tcp){
    if(tcp == "pinch_link" || tcp == "power_link"){
        arm_mover.set_EEF_link(tcp);
    }
    else{
        std::cout << "Wrong tcp link was set." << std::endl;
    }
}

void ArmController::set_grasp(geometry_msgs::Pose pose){
    grasp_pose = pose;
    pre_grasp_pose = compute_pre_pose(grasp_pose);
}

void ArmController::set_place_pose(geometry_msgs::Pose pose){
    place_pose = pose;
    pre_place_pose = compute_pre_pose(pose);
}

void ArmController::approach_grasp(){
    ROS_INFO_STREAM("New plan: PRE GRASP POSE");

    // geometry_msgs::Pose target_pose = arm_mover.getCurrentPose();
    // target_pose.position.z -= 0.2;
    // arm_mover.apend_waypt(target_pose);

    // arm_mover.apend_waypt(target_pose);  
    // arm_mover.apend_waypt(pre_grasp_pose);  

    // arm_mover.build_cart_plan();
    // arm_mover.clear_waypt();

    arm_mover.moveTo(pre_grasp_pose);
}

void ArmController::move_to_g_pose(){
    ROS_INFO_STREAM("New plan: GRASP POSE");
    // geometry_msgs::Pose target_pose = arm_mover.getCurrentPose();
    // arm_mover.apend_waypt(target_pose);
    geometry_msgs::Pose target_pose = arm_mover.getCurrentPose();
    target_pose.position.z -= 0.2;
    arm_mover.apend_waypt(target_pose);
    arm_mover.apend_waypt(pre_grasp_pose);
    arm_mover.apend_waypt(grasp_pose);
    arm_mover.build_cart_plan();
    arm_mover.clear_waypt();

    // arm_mover.build_cart_plan();
    // arm_mover.clear_waypt();
}

void ArmController::move_to_place_pose(){
    ROS_INFO_STREAM("New plan: PLACE POSE");
    // std::vector<double> rotated_joints = arm_mover.getCurrentJointState();
    // rotated_joints[0] += M_PI_2;
    // arm_mover.moveTo(rotated_joints);

    geometry_msgs::Pose target_pose = arm_mover.getCurrentPose();

    // arm_mover.apend_waypt(target_pose);
    arm_mover.apend_waypt(pre_grasp_pose);
    arm_mover.apend_waypt(place_pose);
    arm_mover.build_cart_plan();
    arm_mover.clear_waypt();

    // arm_mover.moveTo(place_pose);
}

void ArmController::pick_up(){
    ROS_INFO_STREAM("New plan: PICK UP");
    // geometry_msgs::Pose target_pose = arm_mover.getCurrentPose();
    // arm_mover.apend_waypt(target_pose);
    arm_mover.moveTo(pre_grasp_pose);
    // arm_mover.build_cart_plan();
    // arm_mover.clear_waypt();
}

void ArmController::move_home(){
    ROS_INFO_STREAM("New plan: HOMENIG");
    // geometry_msgs::Pose target_pose = arm_mover.getCurrentPose();
    // arm_mover.apend_waypt(target_pose);
    // arm_mover.apend_waypt(pre_place_pose);
    // arm_mover.build_cart_plan();
    // arm_mover.clear_waypt();
    arm_mover.moveHome();
    arm_mover.printCurrentJointPosition();
}

geometry_msgs::Pose ArmController::get_current_pose(){
    return arm_mover.getCurrentPose();
}

geometry_msgs::Pose ArmController::compute_pre_pose(geometry_msgs::Pose final_pose){
    geometry_msgs::Pose pre_pose = final_pose;

    Eigen::Vector3d offsets = compute_normal_offset(final_pose.orientation);
    pre_pose.position.x += offsets[0];
    pre_pose.position.y += offsets[1];
    pre_pose.position.z += offsets[2];

    return pre_pose;
}

Eigen::Vector3d ArmController::compute_normal_offset(geometry_msgs::Quaternion orient){
    Eigen::Quaterniond quad = orient_msg_to_eigen(orient);

    Eigen::Vector3d normal_offsets(0, 0, -0.05), oriented_offsets;

    oriented_offsets = quad.matrix()*normal_offsets;
    return oriented_offsets;
}

Eigen::Quaterniond ArmController::orient_msg_to_eigen(
        geometry_msgs::Quaternion orient){
    Eigen::Quaterniond quad;
    quad.x() = orient.x;
    quad.y() = orient.y;
    quad.z() = orient.z;
    quad.w() = orient.w;
    return quad;
}