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
    ros::param::get(path + "left", left_pose);
    ros::param::get(path + "right", right_pose);
    }

ArmController::~ArmController(){
}
void ArmController::approach_grasp(){
    // move_to_pre_pose();

    geometry_msgs::Pose target_pose = arm_mover.getCurrentPose();
    target_pose.position.z -= 0.2;

    arm_mover.apend_waypt(target_pose);
    arm_mover.apend_waypt(pre_grasp_pose);
    arm_mover.apend_waypt(grasp_pose);

    arm_mover.build_cart_plan();
    arm_mover.clear_waypt();
}

void ArmController::move_to_pre_pose(){
    right_gr_flag = check_right_grasp();
    if(right_gr_flag){
        ROS_INFO_STREAM("Approaching right pre grasp pose");
        arm_mover.moveTo(right_pose);
    }
    else{
        ROS_INFO_STREAM("Approaching left pre grasp pose");

        arm_mover.moveTo(left_pose);
    }
}

void ArmController::set_grasp(geometry_msgs::Pose pose){
    grasp_pose = pose;
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
    Eigen::Quaterniond quad = orient_msg_to_eigen(orient);

    Eigen::Vector3d normal_offsets(0, 0, -0.07);
    std::cout << "The offsets in target grasp coordinates are:\n"
        << normal_offsets << std::endl;

    offsets = quad.matrix()*normal_offsets;

    std::cout << "The offsets in abs coordinates are:\n"
        << offsets << std::endl;
}

bool ArmController::check_right_grasp(){
    Eigen::Quaterniond quad = orient_msg_to_eigen(grasp_pose.orientation);
    
    Eigen::Vector3d Y_axis, obj_vect, y_proj, obj_proj;
    Y_axis = quad.matrix().col(1);
    obj_vect = -pose_msg_to_eigen(grasp_pose.position);

    y_proj = projectVectorOntoPlane(Y_axis);
    obj_proj = projectVectorOntoPlane(obj_vect);
    
    // Calculate the cross product of the two projected vectors
    Eigen::Vector3d cross_prod = y_proj.cross(obj_proj);

    // Get the direction of the cross product along the plane normal
    Eigen::Vector3d Z_plane_n(0.0, 0.0, 1.0);
    double cross_dir = cross_prod.dot(Z_plane_n);

    // Determine the sign of the angle
    bool right_flag = (cross_dir >= 0);
    ROS_INFO_STREAM(right_flag);
    return right_flag;
}

Eigen::Quaterniond ArmController::orient_msg_to_eigen(geometry_msgs::Quaternion orient){
    Eigen::Quaterniond quad;
    quad.x() = orient.x;
    quad.y() = orient.y;
    quad.z() = orient.z;
    quad.w() = orient.w;
    return quad;
}
Eigen::Vector3d ArmController::pose_msg_to_eigen(geometry_msgs::Point position){
    Eigen::Vector3d vect;
    vect << 
        position.x,
        position.y,
        position.z;
    
    return vect;
}

Eigen::Vector3d ArmController::projectVectorOntoPlane(Eigen::Vector3d v){
    Eigen::Vector3d Z_plane_n(0.0, 0.0, 1.0);
    double dot_prod = v.dot(Z_plane_n);
    double mag_n_sq = Z_plane_n.squaredNorm();
    Eigen::Vector3d v_parallel = (dot_prod / mag_n_sq) * Z_plane_n;
    Eigen::Vector3d v_proj = v - v_parallel;
    return v_proj;
}