#include <pandaqb_movegroup_control/Grasp/HandControllerBase.h>

HandControllerBase::HandControllerBase() {
    // Initialize ROS node handle
    ros::NodeHandle nh;

    // Create publisher to send trajectory messages
    pub_ = nh.advertise<trajectory_msgs::JointTrajectory>(
            "/qbhand2m1/control/qbhand2m1_motor_positions_trajectory_controller/command", 10);
}
HandControllerBase::~HandControllerBase(){}

void HandControllerBase::sendTrajectory(double pos1, double pos2, double duration) { // seconds
    // Create JointTrajectory message
    trajectory_msgs::JointTrajectory traj_msg;
    traj_msg.header.seq = 0;
    traj_msg.header.stamp = ros::Time::now();
    traj_msg.header.frame_id = "";

    // Add joint names
    traj_msg.joint_names.push_back("qbhand2m1_motor_1_joint");
    traj_msg.joint_names.push_back("qbhand2m1_motor_2_joint");

    // Add trajectory point
    trajectory_msgs::JointTrajectoryPoint point;
    point.positions.push_back(pos1);
    point.positions.push_back(pos2);
    point.velocities.push_back(0.0);
    point.velocities.push_back(0.0);
    point.accelerations.push_back(0.0);
    point.accelerations.push_back(0.0);
    point.effort.push_back(0.0);
    point.effort.push_back(0.0);
    point.time_from_start = ros::Duration(duration);
    traj_msg.points.push_back(point);

    // Publish message
    pub_.publish(traj_msg);
}