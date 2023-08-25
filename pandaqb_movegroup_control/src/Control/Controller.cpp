#include <pandaqb_movegroup_control/Control/Controller.h>

Controller::Controller(ros::NodeHandle *nh): GraspListener(nh),
        arm_controller(), eef_controller(nh){
    grasp_server = nh->advertiseService(
        "gr_exec", &Controller::callback, this);
}

Controller::~Controller(){
}

bool Controller::callback(std_srvs::SetBoolRequest &req, 
        std_srvs::SetBoolResponse &res){

    geometry_msgs::Pose target_pose = get_grasp_pose();

    arm_controller.set_grasp(target_pose);
    eef_controller.open();
    arm_controller.approach_grasp();
    eef_controller.close_hand();
    arm_controller.pick_up();

    res.success = true;
    return true;
}