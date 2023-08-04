#include <pandaqb_movegroup_control/Control/HandController.h>

HandController::HandController():
        open_flag(false), duration(1.0), hand_mover("hand"){

    open();
}

HandController::~HandController(){
}

void HandController::grasp(bool pw_gr_flag){
    if(pw_gr_flag){
        power();
    }
    else{
        pinch();
    }
}

void HandController::pinch(){
    ROS_INFO_STREAM("STARTING PINCH PROCESS");

    std::vector<double> motor_pinch{0.8, 0.8};  // from 0 to 1
    send_trajectory(motor_pinch);

    open_flag = false;
}
void HandController::power(){
    std::vector<double> motor_power{0, 0.9};
    ROS_INFO_STREAM("STARTING POWER PROCESS");

    send_trajectory(motor_power);

    open_flag = false;
}
void HandController::open(){
    std::vector<double> motor_open{0, 0};
    ROS_INFO_STREAM("STARTING OPEN PROCESS");

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
    ROS_INFO_STREAM("Hand will move to: " << motor_command[0] 
        << ", " << motor_command[1]);
    hand_mover.moveTo(motor_command);
}