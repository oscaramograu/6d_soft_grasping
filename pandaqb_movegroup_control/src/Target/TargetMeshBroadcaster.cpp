#include <pandaqb_movegroup_control/Target/TargetMeshBroadcaster.h>

TargetMeshBr::TargetMeshBr(const std::string& object_name): 
    TargetObject(object_name){
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

void TargetMeshBr::build_pose(){
    pose.position.x = position.getX();
    pose.position.y = position.getY();
    pose.position.z = position.getZ();

    pose.orientation.w = rotation.getW();
    pose.orientation.x = rotation.getX();
    pose.orientation.y = rotation.getY();
    pose.orientation.z = rotation.getZ();
}

void TargetMeshBr::add_target_mesh(){
    listen();
    build_pose();
    add_to_world(pose);
}