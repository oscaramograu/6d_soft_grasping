#include <tf/transform_listener.h>
#include <ros/ros.h>
#include "std_msgs/Bool.h"

class GraspListener{
private:
    tf::TransformListener listener;
    tf::StampedTransform transform;

    ros::Subscriber pow_gr_sub;
    
    bool power_gr;
    geometry_msgs::Pose target_pose;

    geometry_msgs::Pose tf_to_pose(tf::StampedTransform transform);

    void callback(const std_msgs::Bool::ConstPtr& msg);
    
public:
    GraspListener(ros::NodeHandle *nh);
    ~GraspListener();

    geometry_msgs::Pose get_grasp_pose();
    bool get_power_gr_flag();

    void build_gr_pose();
};