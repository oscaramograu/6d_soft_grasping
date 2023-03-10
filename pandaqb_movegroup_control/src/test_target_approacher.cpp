#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <pandaqb_movegroup_control/ControllerAPIs/target_aproacher.h>

int main(int argc, char** argv){
    // Initialize the node
    ros::init(argc, argv, "test_target_approacher_node");
    
    // ROS spinning must be running for the MoveGroupInterface to get information about the robot's state.
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // Instatiate the targetApproacher object
    TargetApproacher targetApproacher;

    // Rotate the arm arround z and move down 5cm
    // targetApproacher.approach();

    std::vector<geometry_msgs::Pose> waypoints;
    geometry_msgs::Pose target_pose1, target_pose2, target_pose3, target_pose4;
    target_pose1 = targetApproacher.getCurrentPose();
    target_pose1.position.z -= .1;

    target_pose2 = target_pose1;
    target_pose2.position.y -= .1;

    target_pose3 = target_pose2;
    target_pose3.position.y += .1;
    target_pose3.position.z += .1;

    waypoints.push_back(targetApproacher.getCurrentPose());
    waypoints.push_back(target_pose1);
    waypoints.push_back(target_pose2);
    waypoints.push_back(target_pose3);

    targetApproacher.cartesianSpaceMotion(waypoints);

    target_pose4 = target_pose3;
    target_pose4.position.y -= .3;
    targetApproacher.moveArmToPose(target_pose4);
    return 0;
}