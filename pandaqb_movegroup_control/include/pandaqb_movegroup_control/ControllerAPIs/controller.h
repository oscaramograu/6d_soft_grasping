#include <pandaqb_movegroup_control/ControllerAPIs/grasper.h>
#include <pandaqb_movegroup_control/RequestGrasp.h>


class Controller {
public:
    Controller();
    ~Controller();

    void routine();

private:
    // Make the request of the grasp parameters to the camera node
    void RequestGrasp();

    // Client server communication attributes
    ros::NodeHandle nh_;
    ros::ServiceClient RequestClient;

    pandaqb_movegroup_control::RequestGrasp RequestGraspSrv;

    // Grasper class instance
    Grasper grasper;
};