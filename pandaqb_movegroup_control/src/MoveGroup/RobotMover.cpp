#include <pandaqb_movegroup_control/MoveGroup/RobotMover.h>

RobotMover::RobotMover(): 
    eef_frame("qbhand2m_palm_link"), GroupMover("panda_arm"){
    move_group_->setEndEffectorLink(eef_frame);
    ROS_INFO_STREAM("The end effector frame is: " << move_group_->getEndEffectorLink());
}

RobotMover::~RobotMover(){
}

void RobotMover::move(geometry_msgs::Pose targetGrasp){
    geometry_msgs::Pose target_pose;
    target_pose = target_to_EEF(targetGrasp);

    ROS_INFO_STREAM("The target pose is:  \n" << target_pose);
    ROS_INFO_STREAM("The target grasp pose was:  \n" << targetGrasp);
    visual_tools->publishAxisLabeled(targetGrasp, "target grasp");

    moveTo(target_pose);
}

geometry_msgs::Pose RobotMover::target_to_EEF(geometry_msgs::Pose targetGrasp){
    tf2::Transform targetGrasp_tf;
    tf2::fromMsg(targetGrasp, targetGrasp_tf);

    tf2::Vector3 position = targetGrasp_tf.getOrigin();
    tf2::Quaternion orientation = targetGrasp_tf.getRotation();

    tf2::Matrix3x3 rotation_matrix(orientation);
    tf2::Vector3 normal(0.0, 0.0, 1.0);
    normal = rotation_matrix * normal;

    normal.normalize();

    position += normal * 0.05;
    orientation *= -1;
    
    geometry_msgs::Pose EEF_pose;
    EEF_pose.position.x = position.x();
    EEF_pose.position.y = position.y();
    EEF_pose.position.z = position.z();
    EEF_pose.orientation = tf2::toMsg(orientation);

    return EEF_pose;
}