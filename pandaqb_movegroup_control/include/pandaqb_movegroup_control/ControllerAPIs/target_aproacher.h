#include <pandaqb_movegroup_control/GraspingService.h>
#include <pandaqb_movegroup_control/ControllerAPIs/MoveIt/move_group.h>
#include <pandaqb_movegroup_control/CameraAPIs/image_handler.h>


class TargetApproacher: public MoveGroup{
public:
    TargetApproacher();
    ~TargetApproacher();

    // Function to request the target pose and approach it
    void approach();

private:
    // Function to make the request of the grasp pose in an image
    void requestGraspPose();

    // Functions to get the target pose of the EEF based on the grasp
    void getTargetPose();
    void getTragetOrientation();


    // Image Handler instance
    ImageHandler ih;
    std::string image_path;

    // Used for the client service communication
    ros::NodeHandle nh_;
    ros::ServiceClient graspingClient;
    pandaqb_movegroup_control::GraspingService srv;

    // Recieved from DL model in requestPose()
    float theta, w;
    geometry_msgs::Pose::_position_type graspPose;

    // Used to move to approach to the target pose in approach()
    float z_offset;
    geometry_msgs::Pose targetPose;
};