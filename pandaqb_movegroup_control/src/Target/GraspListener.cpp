#include  <pandaqb_movegroup_control/Target/GraspListener.h>

GraspListener::GraspListener(ros::NodeHandle *nh){
    gr_param_sub = nh->subscribe("grasp_params", 100, 
        &GraspListener::gr_param_callback, this);

        pandaqb_movegroup_control::Grasp::ConstPtr msg = 
        ros::topic::waitForMessage<pandaqb_movegroup_control::Grasp>(
            "/grasp_params");
};

GraspListener::~GraspListener(){
};

geometry_msgs::Pose GraspListener::get_pose_from_tf(std::string tf_frame){
    try {
        listener.waitForTransform("/panda_link0", tf_frame, 
            ros::Time(0), ros::Duration(50.0));
        listener.lookupTransform("/panda_link0", tf_frame, 
            ros::Time(0), transform);
    } catch (tf::TransformException ex) {
        ROS_ERROR("%s",ex.what());
    }

    return tf_to_pose(transform);
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

void GraspListener::gr_param_callback(
    const pandaqb_movegroup_control::Grasp::ConstPtr& msg){
    sinergies = {msg->sinergies[0], msg->sinergies[1]};
    width = msg->width;
}

geometry_msgs::Pose GraspListener::get_grasp_pose(){
    return get_pose_from_tf("/target_grasp");;
}

geometry_msgs::Pose GraspListener::get_place_pose(){
    return get_pose_from_tf("/place_pose");
}

std::vector<double> GraspListener::get_sinergies(){
    ROS_INFO_STREAM("The grasp sinergies are: " << 
        sinergies[0] << ", " << sinergies[1]);

    return sinergies;
}

float GraspListener::get_width(){
    ROS_INFO_STREAM("The widht is: " << width);
    return width;
}