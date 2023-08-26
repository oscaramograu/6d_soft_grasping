#include <pandaqb_movegroup_control/Control/HandController.h>

HandController::HandController(): GroupMover("hand"){
    open_flag = false; 
    duration = 1.0;
}

HandController::~HandController(){
}

void HandController::print_joints(){
    printCurrentJointPosition();
}

void HandController::grasp(std::vector<float> sinergies){
    ROS_INFO_STREAM("HAND GRASP");
    std::vector<double> motor_pinch{sinergies[1], sinergies[0]};  // from 0 to 1
    send_trajectory(motor_pinch);
    open_flag = false;
}

void HandController::open(){
    std::vector<double> motor_open{0, 0};
    ROS_INFO_STREAM("HAND OPEN");
    send_trajectory(motor_open);
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

void HandController::send_trajectory(std::vector<double> motor_command){
    moveTo(motor_command);
}