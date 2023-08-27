#ifndef GRASPLISTENER
#define GRASPLISTENER

#include <tf/transform_listener.h>
#include <ros/ros.h>
#include "pandaqb_movegroup_control/Grasp.h"

class GraspListener{
private:
    tf::TransformListener listener;
    tf::StampedTransform transform;

    ros::Subscriber gr_param_sub;
    
    float width;
    std::vector<double> sinergies;

    geometry_msgs::Pose tf_to_pose(
        tf::StampedTransform transform);

    void gr_param_callback(
        const pandaqb_movegroup_control::Grasp::ConstPtr& msg);

    geometry_msgs::Pose get_pose_from_tf(std::string tf_frame);

public:
    GraspListener(ros::NodeHandle *nh);
    ~GraspListener();

    geometry_msgs::Pose get_grasp_pose();
    geometry_msgs::Pose get_place_pose();
    std::vector<double> get_sinergies();
    float get_width();

};
#endif