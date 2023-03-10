#include <ros/ros.h>
#include <pandaqb_movegroup_control/CameraAPIs/camera.h>

int main(int argc, char** argv){
    // Initialize the node
    ros::init(argc, argv, "camera_node");

    Camera camera;
    
    ros::spin();

    return 0;
}