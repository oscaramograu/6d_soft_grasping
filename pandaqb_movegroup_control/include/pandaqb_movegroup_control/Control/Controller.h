#include <pandaqb_movegroup_control/Control/ArmController.h>
#include <pandaqb_movegroup_control/Control/EEFController.h>
#include <pandaqb_movegroup_control/Target/GraspListener.h>
#include <std_msgs/Bool.h>


class Controller: public GraspListener{
public:
    Controller(ros::NodeHandle *nh);
    ~Controller();
    void start_new_routine();

private:    
    void pick_and_place_routine();
    void pick_routine();

    ArmController arm_controller;
    EEFController eef_controller;

    geometry_msgs::Pose target_pose, place_pose;
};  