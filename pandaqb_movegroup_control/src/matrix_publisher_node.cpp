#include <ros/ros.h>
#include <pandaqb_movegroup_control/MatrixManipulation/obj_w_Rt_publisher.h>

int main(int argc, char** argv){
    // Initialize the node
    ros::init(argc, argv, "Rt_publisher_node");

    ObjWorld_RtPublisher ObjWorld_pub;

    ros::spin();

    return 0;    
}