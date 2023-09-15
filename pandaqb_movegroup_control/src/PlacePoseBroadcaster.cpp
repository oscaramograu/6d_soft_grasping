#include <ros/ros.h>
#include <pandaqb_movegroup_control/Target/GraspListener.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <tf/transform_broadcaster.h>
#include <random>

geometry_msgs::Pose place_pose;
tf::Transform place_tf;

// Eigen::Quaterniond compute_z_rot(
//         geometry_msgs::Pose grasp_pose1, geometry_msgs::Pose grasp_pose2){
//     Eigen::Vector3d pick_pose_v(
//         grasp_pose1.position.x, grasp_pose1.position.y, grasp_pose1.position.z);
//     Eigen::Vector3d place_pose_v(
//         grasp_pose2.position.x, grasp_pose2.position.y, grasp_pose2.position.z);

//     double angle = std::acos(pick_pose_v.dot(place_pose_v) 
//                     / (pick_pose_v.norm() * place_pose_v.norm()));

//     Eigen::Quaterniond z_rot(Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ()));
//     return z_rot;
// } 

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

geometry_msgs::Pose::_orientation_type compute_place_or(){
    geometry_msgs::Pose final_pose;
    Eigen::Quaterniond base_or, rot_y, rot_z, place_pose_or_1, place_pose_or_2;

    base_or.z() = 1;

    double angle_radians_y = M_PI;
    // double angle_radians_y = 0;

    rot_y.w() = cos(angle_radians_y/2);
    rot_y.y() = -sin(angle_radians_y/2);

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dist_rot_Z(- M_PI_4, M_PI_4);
    double angle_radians_z = M_PI_2 + dist_rot_Z(gen);
    // double angle_radians_z = 0;

    rot_z.w() = cos(angle_radians_z/2);
    rot_z.z() = -sin(angle_radians_z/2);

    place_pose_or_1 = rot_y * base_or;
    place_pose_or_2 = rot_z * place_pose_or_1;

    return eigen_to_orient_msg(place_pose_or_2);
}

void init_place_pose(geometry_msgs::Pose &place_pose){
    double corner_x = 0.23, corner_y = -0.11, l_x = 0.405, l_y = 0.255;
    double random_off_x, random_off_y, margin(0.13);

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dist_x(corner_x + margin, corner_x + l_x - margin);
    std::uniform_real_distribution<double> dist_y(corner_y + margin, corner_y + l_y - margin);

    place_pose.position.x = dist_x(gen);
    place_pose.position.y = dist_y(gen);
    place_pose.position.z = 0.155 + 0.15;

    place_pose.orientation = compute_place_or();
}  

int main(int argc, char **argv){
    ros::init(argc, argv, "place_pose_broadcaster_node");
    ros::NodeHandle n;
    // GraspListener gr_listener(&n);
    tf::TransformBroadcaster broadcaster;

    init_place_pose(place_pose);
    // std::cout << place_pose << std::endl;
    ros::Rate loop_rate(10); // Publish at 10 Hz

    while (ros::ok()) {
        place_tf = geometry_msgs_to_tf(place_pose);
        broadcaster.sendTransform(tf::StampedTransform(place_tf, ros::Time::now(), "panda_link0", "place_pose"));
        
        loop_rate.sleep();
    }

    ros::spin();
    ros::shutdown();
    return 0;
}