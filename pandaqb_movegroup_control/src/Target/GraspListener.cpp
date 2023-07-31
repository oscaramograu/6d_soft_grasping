#include  <pandaqb_movegroup_control/Target/GraspListener.h>

GraspListener::GraspListener(ros::NodeHandle *nh){
    pow_gr_sub = nh->subscribe("power_gr", 100, &GraspListener::callback, this);
};

GraspListener::~GraspListener(){
};

void GraspListener::build_gr_pose(){
    try {
        listener.waitForTransform("/panda_link0", "/target_grasp", 
            ros::Time(0), ros::Duration(50.0));
        listener.lookupTransform("/panda_link0", "/target_grasp", 
            ros::Time(0), transform);
    } catch (tf::TransformException ex) {
        ROS_ERROR("%s",ex.what());
    }

    target_pose = tf_to_pose(transform);
}

geometry_msgs::Pose GraspListener::tf_to_pose(tf::StampedTransform transform){
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

void GraspListener::callback(const std_msgs::Bool::ConstPtr& msg){
    power_gr = msg->data;
}

geometry_msgs::Pose GraspListener::get_grasp_pose(){
    build_gr_pose();
    ROS_INFO_STREAM(target_pose);
    return target_pose;
}

bool GraspListener::get_power_gr_flag(){
    std_msgs::Bool::ConstPtr msg = ros::topic::waitForMessage
        <std_msgs::Bool>("/power_gr");
    ROS_INFO_STREAM(power_gr);

    return power_gr;
}