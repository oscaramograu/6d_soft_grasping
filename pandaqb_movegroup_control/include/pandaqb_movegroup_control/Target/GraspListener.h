#ifndef GRASPLISTENER
#define GRASPLISTENER

#include <tf/transform_listener.h>
#include <ros/ros.h>
#include "std_msgs/Float32.h"
#include "pandaqb_movegroup_control/Sinergies.h"

class GraspListener{
private:
    tf::TransformListener listener;
    tf::StampedTransform transform;

    ros::Subscriber width_sub, sinergies_sub;
    
    float width;
    std::vector<float> sinergies;

    geometry_msgs::Pose target_pose;

    geometry_msgs::Pose tf_to_pose(tf::StampedTransform transform);

    void width_callback(const std_msgs::Float32::ConstPtr& msg);
    void sinergies_callback(
        const pandaqb_movegroup_control::Sinergies::ConstPtr& msg);

public:
    GraspListener(ros::NodeHandle *nh);
    ~GraspListener();

    geometry_msgs::Pose get_grasp_pose();
    float get_width();
    std::vector<float> get_sinergies();

    void build_gr_pose();
};
#endif