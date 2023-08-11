#include <pandaqb_movegroup_control/Control/GripperController.h>

GripperController::GripperController(): GroupMover("gripper"){
    set_EEF_link("tcp_frame");
    open_gripper();
}

GripperController::~GripperController(){
}

void GripperController::close_gripper(float width){
    ROS_INFO_STREAM("STARTING GRASPING PROCESS");
    std::vector <double> widht_v{width/2, width/2};
    moveTo(widht_v);
}

void GripperController::open_gripper(){
    ROS_INFO_STREAM("STARTING GRASPING PROCESS");
    moveTo(open_with);
}