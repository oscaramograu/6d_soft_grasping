#include <ros/ros.h>
// #include <pandaqb_movegroup_control/Target/TargetMeshBroadcaster.h>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char** argv){
    // Initialize the node
    // ros::init(argc, argv, "controller_node");
    // Create an instance of the ObjectBase class
    // TargetMeshBr object("cpsduck");
    // object.add_target_mesh();
    // Define the pose for adding the object to the world
    // geometry_msgs::Pose pose;
    // pose.orientation.w = 1;
    // pose.orientation.x = 0;
    // pose.orientation.y = 0;
    // pose.orientation.z = 0;
    // pose.position.x =  0.3;
    // pose.position.y =  -0.25;
    // pose.position.z =  0.0;
    // Set the pose values as needed

    // Add the object to the world
    // object.add_to_world(pose);

    // Spin and process ROS callbacks
    ros::spin();
    // // ROS spinning must be running for the MoveGroupInterface to get information about the robot's state.
    // ros::AsyncSpinner spinner(1);
    // spinner.start();

// ################################################################################################################
    //GROUP MOVE  TESTS
// ################################################################################################################
    // GroupMover arm_mover("panda_arm");
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