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
    geometry_msgs::Pose target_pose;
    // tf::StampedTransform transform;
    // tf::TransformListener listener;

    // try {
    //     listener.waitForTransform("/panda_link0", "/target_grasp", 
    //         ros::Time(0), ros::Duration(50.0));
    //     listener.lookupTransform("/panda_link0", "/target_grasp", 
    //         ros::Time(0), transform);
    // } catch (tf::TransformException ex) {
    //     ROS_ERROR("%s",ex.what());
    // }

    // target_pose = tf_to_pose(transform);
    // ROS_INFO_STREAM("Target pose: " << target_pose);


    GroupMover arm("arm");
    bool theta = M_PI;
    arm.set_EEF_link("qbhand2m1_end_effector_link");

    target_pose = arm.getCurrentPose();
    target_pose.position.z -= 0.2;

    arm.moveTo(target_pose);


    // ArmController ac;
    // HandController hc;

    // ac.set_grasp_pose(target_pose);
    // ac.approach_grasp();

    // hc.power();
    // ac.pick_up();





// // grasp detection pose
// // 0.42754; 0.18872; 0.25229
// // -0.32689; 0.8196; 0.32044; -0.34455
//     pose.position.x -= 0.1;
//     pose.position.z += 0.1;
//     arm_mover.moveTo(pose);

//     pose.position.x += 0.1;
//     pose.position.z -= 0.09;
//     arm_mover.moveTo(pose);

    // ROS_INFO_STREAM( "eef link: " << arm_mover.getCurrentPose());


    





    // static tf::TransformBroadcaster br;
    // while (true)
    // {
    //     br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "panda_link0", "eef"));
    // }
    


    // geometry_msgs::Pose pose = arm_mover.getCurrentPose();
    // pose.position.z += 0.1;

    // arm_mover.moveTo(pose);
    // arm_mover.moveHome();

    // GroupMover hand_mover("qb_hand");
    // hand_mover.printCurrentJointPosition();
    // std::vector<double> closed_hand = {0.0};
    // hand_mover.moveTo(closed_hand);

    // RobotMover robot_mover;
    // geometry_msgs::Pose target_pose = robot_mover.getCurrentPose();
    // target_pose.position.x += 0.1;
    // target_pose.position.y += 0.1;
    // target_pose.position.z -= 0.1;   
    // target_pose.orientation.x = 5.5511e-17;
    // target_pose.orientation.y = -0.2474;
    // target_pose.orientation.z = -5.5511e-17;
    // target_pose.orientation.w = 0.96891;
    
    // robot_mover.move(target_pose);

// ################################################################################################################
    //CONTROLLER  TESTS
// ################################################################################################################
    // Controller controller;
    // controller.routine();
    
// ################################################################################################################
    //GRASPER  TESTS
// ################################################################################################################
    // std::vector<float> graspPosition;
    // float w, theta;

    // ros::param::get("move_group_params/grasp_position", graspPosition);
    // ros::param::get("move_group_params/theta", theta);
    // ros::param::get("move_group_params/w", w);    
    
    // Manipulator targetApproacher;

    // targetApproacher.setGraspParams(theta,w,graspPosition);

    // targetApproacher.moveToHomePose();
    // targetApproacher.approach();
    // targetApproacher.moveToHomePose();

// ################################################################################################################
    //MOVEGROUP TESTS:
// ################################################################################################################
    // MoveGroup mg;
    // mg.printCurrentJointPosition();
    // std::vector<geometry_msgs::Pose> waypoints;
    // geometry_msgs::Pose target_pose1, target_pose2, target_pose3, target_pose4;
    // target_pose1 = targetApproacher.getCurrentPose();
    // target_pose1.position.z -= .1;

    // target_pose2 = target_pose1;
    // target_pose2.position.y -= .1;

    // target_pose3 = target_pose2;
    // target_pose3.position.y += .1;
    // target_pose3.position.z += .1;

    // waypoints.push_back(targetApproacher.getCurrentPose());
    // waypoints.push_back(target_pose1);
    // waypoints.push_back(target_pose2);
    // waypoints.push_back(target_pose3);

    // targetApproacher.cartesianSpaceMotion(waypoints);

    // target_pose4 = target_pose3;
    // target_pose4.position.y -= .3;
    // targetApproacher.moveArmToPose(target_pose4);

// ################################################################################################################
    //HANDGROUP TESTS:
// ################################################################################################################
    // HandGroup hg;
    // hg.grasp(0.3);

    ros::shutdown();
    return 0;
}