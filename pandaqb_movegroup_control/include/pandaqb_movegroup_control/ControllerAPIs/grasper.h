#include <pandaqb_movegroup_control/GraspingService.h>
#include <pandaqb_movegroup_control/ControllerAPIs/move_group.h>
#include <pandaqb_movegroup_control/CameraAPIs/image_handler.h>


class Grasper: public MoveGroup{
public:
    Grasper();
    ~Grasper();

    void setGraspParams(float theta_, float w_, std::vector <float> GraspPoint_);
    void setGraspParams(float theta_, float w_, geometry_msgs::Point graspPoint_);
    
    void approach();

    void grasp();
    
private:
    // Function to make the request of the grasp pose in an image
    void setTragetOrientation();
    void setTargetPosition();

    // Recieved from DL model in the Controller request
    float theta, w;
    geometry_msgs::Pose graspPoint;

    // Used to move to approach to the target pose in approach()
    float z_offset;
    geometry_msgs::Pose targetPose;
};