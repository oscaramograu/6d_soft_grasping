#include  <pandaqb_movegroup_control/Target/GraspListener.h>

GraspListener::GraspListener(ros::NodeHandle *nh){
    width_sub = nh->subscribe("gr_width", 100, &GraspListener::width_callback, this);
    sinergies_sub = nh->subscribe("sinergies", 100, &GraspListener::sinergies_callback, this);
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

void GraspListener::width_callback(const std_msgs::Float32::ConstPtr& msg){
    width = msg->data;
}

void GraspListener::sinergies_callback(
    const pandaqb_movegroup_control::Sinergies::ConstPtr& msg){
    sinergies = {msg->first_sin, msg->second_sin};
}

geometry_msgs::Pose GraspListener::get_grasp_pose(){
    build_gr_pose();
    ROS_INFO_STREAM(target_pose);
    return target_pose;
}

float GraspListener::get_width(){
    std_msgs::Float32::ConstPtr msg = ros::topic::waitForMessage
        <std_msgs::Float32>("/gr_width");
    ROS_INFO_STREAM("The grasp width is: " << width);

    return width;
}

std::vector<float> GraspListener::get_sinergies(){
    pandaqb_movegroup_control::Sinergies::ConstPtr msg = 
        ros::topic::waitForMessage<pandaqb_movegroup_control::Sinergies>(
            "/sinergies");

    ROS_INFO_STREAM("The grasp sinergies are: " << 
        sinergies[0] << ", " << sinergies[1]);

    return sinergies;
}