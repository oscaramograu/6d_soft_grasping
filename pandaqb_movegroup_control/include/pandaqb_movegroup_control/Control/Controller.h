#include <pandaqb_movegroup_control/Control/ArmController.h>
#include <pandaqb_movegroup_control/Control/EEFController.h>
#include <pandaqb_movegroup_control/Target/GraspListener.h>
#include <std_srvs/SetBool.h>


class Controller: public GraspListener{
public:
    Controller(ros::NodeHandle *nh);
    ~Controller();

private:
    bool callback(std_srvs::SetBoolRequest &req, 
        std_srvs::SetBoolResponse &res);

    ros::ServiceServer grasp_server;
    
    ArmController arm_controller;
    EEFController eef_controller;
};  