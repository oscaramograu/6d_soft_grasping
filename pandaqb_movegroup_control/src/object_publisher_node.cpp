#include <ros/ros.h>
#include <pandaqb_movegroup_control/Target/TargetMeshBroadcaster.h>

int main(int argc, char** argv){
    // Initialize the node
    ros::init(argc, argv, "object_publisher_node");

    // Create an instance of the object class
    TargetMeshBr object("cpsduck");

    // Execute main code
    object.add_target_mesh();

    // Spin and process ROS callbacks
    ros::spin();
    ros::shutdown();
    return 0;
}