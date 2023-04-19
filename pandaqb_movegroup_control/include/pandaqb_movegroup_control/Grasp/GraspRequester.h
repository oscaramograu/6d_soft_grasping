#include <ros/ros.h>
#include <pandaqb_movegroup_control/RequestGrasp.h>
#include <pandaqb_movegroup_control/Grasp/Grasp.h>

class GraspRequester {
public:
    GraspRequester();
    ~GraspRequester();

    // Make the request of the grasp parameters to the camera node
    void request();
    Grasp getGrasp();

private:
    // Client server communication attributes
    ros::NodeHandle nh_;
    ros::ServiceClient RequestClient;

    pandaqb_movegroup_control::RequestGrasp RequestGraspSrv;
    Grasp grasp;
};