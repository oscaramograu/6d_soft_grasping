#include <pandaqb_movegroup_control/Target/TargetMeshBroadcaster.h>

TargetMeshBr::TargetMeshBr(const std::string& mesh_path, const std::string& object_name): 
    TargetObject(mesh_path, object_name){
}
TargetMeshBr::~TargetMeshBr(){
}

void TargetMeshBr::listen(){
    tf::StampedTransform transform;
    try {
        listener.waitForTransform("/panda_link0", "/" + object_name_ + "_frame", 
            ros::Time(0), ros::Duration(50.0));
        listener.lookupTransform("/panda_link0", "/" + object_name_ + "_frame", 
            ros::Time(0), transform);
    } catch (tf::TransformException ex) {
        ROS_ERROR("%s",ex.what());
    }

    position = transform.getOrigin();
    rotation = transform.getRotation();
}

void TargetMeshBr::listen2(){
    tf::StampedTransform transform;
    try {
        listener.waitForTransform("/panda_link0", "/target_grasp", 
            ros::Time(0), ros::Duration(50.0));
        listener.lookupTransform("/panda_link0", "/target_grasp", 
            ros::Time(0), transform);
    } catch (tf::TransformException ex) {
        ROS_ERROR("%s",ex.what());
    }

    position = transform.getOrigin();
    rotation = transform.getRotation();
}

void TargetMeshBr::build_pose(){
    pose.position.x = position.getX();
    pose.position.y = position.getY();
    pose.position.z = position.getZ();

    pose.orientation.w = rotation.getW();
    pose.orientation.x = rotation.getX();
    pose.orientation.y = rotation.getY();
    pose.orientation.z = rotation.getZ();
}

void TargetMeshBr::add_target_mesh(bool is_obj){
    if(is_obj){
        listen();
    }
    else{
        listen2();
    }

    build_pose();
    add_to_world(pose);
}