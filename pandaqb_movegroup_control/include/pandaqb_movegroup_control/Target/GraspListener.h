#ifndef GRASPLISTENER
#define GRASPLISTENER

#include <tf/transform_listener.h>
#include <ros/ros.h>
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"

class GraspListener{
private:
    tf::TransformListener listener;
    tf::StampedTransform transform;

    ros::Subscriber pow_gr_sub, width_sub;
    
    bool power_gr;
    float width;
    geometry_msgs::Pose target_pose;

    geometry_msgs::Pose tf_to_pose(tf::StampedTransform transform);

    void flag_callback(const std_msgs::Bool::ConstPtr& msg);
    void width_callback(const std_msgs::Float32::ConstPtr& msg);

public:
    GraspListener(ros::NodeHandle *nh);
    ~GraspListener();

    geometry_msgs::Pose get_grasp_pose();
    bool get_power_gr_flag();
    float get_width();

    void build_gr_pose();
};
#endif