#include <pandaqb_movegroup_control/RequestGrasp.h>
#include <pandaqb_movegroup_control/ControllerAPIs/move_group.h>
#include <pandaqb_movegroup_control/ControllerAPIs/hand_group.h>



class Manipulator: public MoveGroup{
public:
    Manipulator();
    ~Manipulator();

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