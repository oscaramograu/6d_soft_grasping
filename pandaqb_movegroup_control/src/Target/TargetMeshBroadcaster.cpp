#include <pandaqb_movegroup_control/Target/TargetMeshBroadcaster.h>

TargetMeshBr::TargetMeshBr(const std::string& mesh_path, const std::string& object_name): 
    TargetObject(mesh_path, object_name){
}
TargetMeshBr::~TargetMeshBr(){
}

tf::StampedTransform  TargetMeshBr::listen(std::string child, std::string parent = "/panda_link0"){
    tf::StampedTransform transform;
    try {
        listener.waitForTransform(parent, child, ros::Time(0), ros::Duration(50.0));
        listener.lookupTransform(parent, child, ros::Time(0), transform);
    } catch (tf::TransformException ex) {
        ROS_ERROR("%s",ex.what());
    }
    return transform;
}

tf::StampedTransform TargetMeshBr::map_pose(tf::Vector3 orgin_pose, tf::Quaternion origin_rot){
    tf::StampedTransform map = listen("qbhand2m1_end_effector_link", "pinch_link");
    tf::Vector3 offset = map.getOrigin();
    tf::Quaternion rot = map.getRotation();



    tf::StampedTransform mapped_tf;
    mapped_tf.setOrigin(orgin_pose + offset);
    mapped_tf.setRotation(origin_rot*rot);
    return mapped_tf;
}

geometry_msgs::Pose TargetMeshBr::build_pose(tf::Vector3 position, tf::Quaternion rotation){
    geometry_msgs::Pose pose;
    pose.position.x = position.getX();
    pose.position.y = position.getY();
    pose.position.z = position.getZ();

    pose.orientation.w = rotation.getW();
    pose.orientation.x = rotation.getX();
    pose.orientation.y = rotation.getY();
    pose.orientation.z = rotation.getZ();

    return pose;
}

void TargetMeshBr::add_target_mesh(bool is_obj){
    if(is_obj){
        target_tf = listen(object_name_ + "_frame");
    }
    else{
        target_tf = listen("target_grasp");

        // tf::StampedTransform origin_tf = listen("target_grasp");

        // target_tf = map_pose(origin_tf.getOrigin(), origin_tf.getRotation());
    }

    target_pose = build_pose(target_tf.getOrigin(), target_tf.getRotation());
    // std::cout << target_pose << std::endl;

    add_to_world(target_pose);
}