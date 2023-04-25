#include <pandaqb_movegroup_control/MoveGroup/RobotMover.h>

RobotMover::RobotMover(): 
    eef_frame("qbhand2m_palm_link"), GroupMover("panda_arm"){
    move_group_->setEndEffectorLink(eef_frame);
    ROS_INFO_STREAM("The end effector frame is: " << move_group_->getEndEffectorLink());

    collision_obj_primitive.type = collision_obj_primitive.SPHERE;
    collision_obj_primitive.dimensions.resize(1);
    collision_obj_primitive.dimensions[collision_obj_primitive.SPHERE_RADIUS] = 0.05;
    // collision_obj_primitive.type = collision_obj_primitive.BOX;
    // collision_obj_primitive.dimensions.resize(3);
    // collision_obj_primitive.dimensions[collision_obj_primitive.BOX_X] = 0.1;
    // collision_obj_primitive.dimensions[collision_obj_primitive.BOX_Y] = 1.5;
    // collision_obj_primitive.dimensions[collision_obj_primitive.BOX_Z] = 0.5;
}

RobotMover::~RobotMover(){
}

void RobotMover::move(geometry_msgs::Pose targetGrasp){
    geometry_msgs::Pose target_pose;
    target_pose = target_to_EEF(targetGrasp);

    ROS_INFO_STREAM("The target pose is:\n" << target_pose);
    ROS_INFO_STREAM("The target grasp pose was:\n" << targetGrasp);
    visual_tools->publishAxisLabeled(targetGrasp, "target grasp");

    moveTo(target_pose);
}

geometry_msgs::Pose RobotMover::target_to_EEF(geometry_msgs::Pose targetGrasp){
    tf2::Transform targetGrasp_tf;
    tf2::fromMsg(targetGrasp, targetGrasp_tf);

    tf2::Vector3 EEF_position_tf = targetGrasp_tf.getOrigin();
    tf2::Vector3 object_position_tf = targetGrasp_tf.getOrigin();
    tf2::Quaternion orientation = targetGrasp_tf.getRotation();

    tf2::Matrix3x3 rotation_matrix(orientation);
    tf2::Vector3 normal(0.0, 0.0, 1.0);
    normal = rotation_matrix * normal;

    normal.normalize();

    EEF_position_tf += normal * 0.01;
    orientation *= -1;
    
    geometry_msgs::Pose EEF_pose;
    EEF_pose.position.x = EEF_position_tf.x();
    EEF_pose.position.y = EEF_position_tf.y();
    EEF_pose.position.z = EEF_position_tf.z();
    EEF_pose.orientation = tf2::toMsg(orientation);

    object_position_tf -= normal * 0.05;

    geometry_msgs::Pose objectPose;
    objectPose.position.x = object_position_tf.x();
    objectPose.position.y = object_position_tf.y();
    objectPose.position.z = object_position_tf.z();
    add_collision_object("grasp_object", collision_obj_primitive, objectPose);

    return EEF_pose;
}