#include <ros/ros.h>
// #include <pandaqb_movegroup_control/Target/TargetMeshBroadcaster.h>
// #include <pandaqb_movegroup_control/MoveGroup/GroupMover.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <pandaqb_movegroup_control/Control/HandController.h>
#include <pandaqb_movegroup_control/Control/ArmController.h>

void rotate_aroundZ(tf::StampedTransform &tf, float theta){
    tf::Quaternion tf_rot;
    tf_rot.setW(cos(theta/2));
    tf_rot.setX(0);
    tf_rot.setY(0);
    tf_rot.setZ(sin(theta/2));
    
    tf::StampedTransform tf_Z;
    tf_Z.setRotation(tf_rot);
    tf*=tf_Z;
}

void rotate_aroundX(tf::StampedTransform &tf, float theta){
    tf::Quaternion tf_rot;
    tf_rot.setW(cos(theta/2));
    tf_rot.setX(sin(theta/2));
    tf_rot.setY(0);
    tf_rot.setZ(0);
    
    tf::StampedTransform tf_X;
    tf_X.setRotation(tf_rot);
    tf*=tf_X;
}

void offset_Z(tf::StampedTransform &tf, float offset){
    tf::Vector3 tf_pose(0, 0, offset);
    
    tf::StampedTransform tf_Z;
    tf_Z.setOrigin(tf_pose);
    tf*=tf_Z;
}


geometry_msgs::Pose tf_to_pose(tf::StampedTransform transform){
    geometry_msgs::Pose pose;
    tf::Vector3 position = transform.getOrigin();
    pose.position.x = position.getX();
    pose.position.y = position.getY();
    pose.position.z = position.getZ();

    tf::Quaternion rotation = transform.getRotation();
    pose.orientation.w = rotation.getW();
    pose.orientation.x = rotation.getX();
    pose.orientation.y = rotation.getY();
    pose.orientation.z = rotation.getZ();

    return pose;
}

int main(int argc, char** argv){
    // Initialize the node
    ros::init(argc, argv, "controller_node");

    // Spin and process ROS callbacks
    // ros::spin();
    // ROS spinning must be running for the MoveGroupInterface to get information about the robot's state.
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // HandController hc;
    // hc.pinch();



// ################################################################################################################
    //GROUP MOVE  TESTS
// ################################################################################################################
//     GroupMover arm_mover("arm");
//     arm_mover.set_EEF_link("qbhand2m_end_effector_link");
    geometry_msgs::Pose target_pose;
    tf::StampedTransform transform;
    tf::TransformListener listener;

    try {
        listener.waitForTransform("/panda_link0", "/target_grasp2", 
            ros::Time(0), ros::Duration(50.0));
        listener.lookupTransform("/panda_link0", "/target_grasp2", 
            ros::Time(0), transform);
    } catch (tf::TransformException ex) {
        ROS_ERROR("%s",ex.what());
    }

    target_pose = tf_to_pose(transform);
    ROS_INFO_STREAM("Target pose: " << target_pose);

// ================= TESTING ===========================

    // GroupMover arm("panda_arm");
    // arm.set_EEF_link("panda_link8");
    // arm.moveTo(target_pose);

    // bool theta = M_PI;
    // target_pose = arm.getCurrentPose();
    // target_pose.orientation.w = cos(theta/2);
    // target_pose.orientation.x = sin(theta/2);
    // target_pose.orientation.y = 0;
    // target_pose.orientation.z = 0;



// ================= REAL CODE ===========================
    ArmController ac;
    HandController hc;

    ac.set_grasp_pose(target_pose);
    ac.approach_grasp();

    hc.power();
    ac.pick_up();


    ros::shutdown();
    return 0;
}