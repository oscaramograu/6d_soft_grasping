#include <pandaqb_movegroup_control/GraspingService.h>
#include <pandaqb_movegroup_control/CameraAPIs/image_handler.h>
#include <pandaqb_movegroup_control/ControllerAPIs/grasper.h>

class Controller {
public:
    Controller();
    ~Controller();

    void routine();

private:
    // Function to make the request of the grasp pose in an image
    void GraspPoseCallback();

    // Used for client service communication
    ros::NodeHandle nh_;
    ros::ServiceClient RequestClient;
    ros::ServiceServer PoseServer;

    pandaqb_movegroup_control::GraspingService srv;

    // Recieved from DL model in requestPose()
    float theta, w;
    geometry_msgs::Pose::_position_type graspPose;

    // Used to move to approach to the target pose in approach()
    float z_offset;
    geometry_msgs::Pose targetPose;
};