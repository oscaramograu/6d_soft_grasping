#include <ros/ros.h>
#include <pandaqb_movegroup_control/Target/GraspListener.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <tf/transform_broadcaster.h>

geometry_msgs::Pose pick_pose, place_pose;
Eigen::Quaterniond pick_or, place_or, z_rot;
tf::Transform place_tf;

Eigen::Quaterniond compute_z_rot(
        geometry_msgs::Pose grasp_pose1, geometry_msgs::Pose grasp_pose2){
    Eigen::Vector3d pick_pose_v(
        grasp_pose1.position.x, grasp_pose1.position.y, grasp_pose1.position.z);
    Eigen::Vector3d place_pose_v(
        grasp_pose2.position.x, grasp_pose2.position.y, grasp_pose2.position.z);

    double angle = std::acos(pick_pose_v.dot(place_pose_v) 
                    / (pick_pose_v.norm() * place_pose_v.norm()));

    Eigen::Quaterniond z_rot(Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ()));
    return z_rot;
}
void init_place_pose(geometry_msgs::Pose &place_pose){
    place_pose.position.x = 0;
    place_pose.position.y = 0.75;
    place_pose.position.z = 0;
}   

Eigen::Quaterniond orient_msg_to_eigen(geometry_msgs::Pose::_orientation_type geo_msg){
    Eigen::Quaterniond quad_msg;
    quad_msg.w() = geo_msg.w;
    quad_msg.x() = geo_msg.x;
    quad_msg.y() = geo_msg.y;
    quad_msg.z() = geo_msg.z;

    return quad_msg;
}

geometry_msgs::Pose::_orientation_type eigen_to_orient_msg(Eigen::Quaterniond quad){

    geometry_msgs::Pose::_orientation_type orient;
    orient.w = quad.w();
    orient.x = quad.x();
    orient.y = quad.y();
    orient.z = quad.z();
    return orient;
}

tf::Transform geometry_msgs_to_tf(geometry_msgs::Pose pose){
    tf::Vector3 translation(pose.position.x, pose.position.y, pose.position.z);
    tf::Quaternion rotation(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
    return tf::Transform(rotation, translation);
}

int main(int argc, char **argv){
    ros::init(argc, argv, "place_pose_broadcaster_node");
    ros::NodeHandle n;
    GraspListener gr_listener(&n);
    tf::TransformBroadcaster broadcaster;

    ros::Rate loop_rate(10); // Publish at 10 Hz
    while (ros::ok()) {

        pick_pose = gr_listener.get_grasp_pose();
        init_place_pose(place_pose);

        pick_or = orient_msg_to_eigen(pick_pose.orientation);
        z_rot = compute_z_rot(pick_pose, place_pose);
        place_or = z_rot*pick_or;

        place_pose.orientation = eigen_to_orient_msg(place_or);
 
        place_tf = geometry_msgs_to_tf(place_pose);
        broadcaster.sendTransform(tf::StampedTransform(place_tf, ros::Time::now(), "panda_link0", "place_pose"));
        
        loop_rate.sleep();
    }

    ros::spin();
    ros::shutdown();
    return 0;
}