#include <ros/ros.h>
#include <pandaqb_movegroup_control/DetectGrasp.h>
#include <pandaqb_movegroup_control/RequestGrasp.h>

class Camera{
public:
    Camera();
    ~Camera();

private:
    bool RequestGraspCallback(pandaqb_movegroup_control::RequestGrasp::Request &req,
        pandaqb_movegroup_control::RequestGrasp::Response &resp);

    void setFakeGraspParams();

    // Client server communication attributes
    ros::NodeHandle nh_;

    ros::ServiceServer RequestGraspServer;
    ros::ServiceClient DetectClient;

    pandaqb_movegroup_control::RequestGrasp RequestGraspSrv;
    pandaqb_movegroup_control::DetectGrasp DetectGraspSrv;

    // Attributes to send to the Controller node
    float theta, w;
    geometry_msgs::Point graspPoint;

    // Attributes to send to the NN node
    //RGB-D ros image
};