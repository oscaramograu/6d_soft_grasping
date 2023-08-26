#include <pandaqb_movegroup_control/Control/HandController.h>

HandController::HandController(): GroupMover("hand"){
    open_flag = false; 
}

HandController::~HandController(){
}

void HandController::print_joints(){
    printCurrentJointPosition();
}

void HandController::grasp(std::vector<double> sinergies){
    ROS_INFO_STREAM("HAND GRASP");
    std::vector<double> hand_joints = {sinergies[1], sinergies[0]};
    moveTo(hand_joints);
    open_flag = false;
}

void HandController::open(){
    std::vector<double> hand_joints{0, 0};
    ROS_INFO_STREAM("HAND OPEN");
    moveTo(hand_joints);
    open_flag = true;
}

bool HandController::is_open(){
    if(open_flag){
        ROS_INFO_STREAM("The hand is open");
    }    
    else{
        ROS_INFO_STREAM("The hand is closed");
    }

    return open_flag;
}