#include <pandaqb_movegroup_control/Grasp/HandController.h>

GraspExecuter::GraspExecuter():  HandControllerBase(),
        open_flag(false), duration(1.0){
    motor_pinch.assign(0.1, 0.5); // Values need to be tested
    motor_open.assign(0.0, 0.0);
    motor_power.assign(0.3, 0.3);

    open();
}

GraspExecuter::~GraspExecuter(){
}

void GraspExecuter::pinch(){
    ROS_INFO_STREAM("STARTING PINCH PROCESS");
    sendTrajectory(motor_pinch[0], motor_pinch[1], duration);

    open_flag = false;
}
void GraspExecuter::power(){
    ROS_INFO_STREAM("STARTING POWER PROCESS");
    sendTrajectory(motor_power[0], motor_power[1], duration);

    open_flag = false;
}
void GraspExecuter::open(){
    ROS_INFO_STREAM("STARTING OPEN PROCESS");
    sendTrajectory(motor_open[0], motor_open[1], duration);

    open_flag = true;
}

bool GraspExecuter::is_open(){
    if(open_flag){
        ROS_INFO_STREAM("The hand is open");
    }    
    else{
        ROS_INFO_STREAM("The hand is closed");
    }

    return open_flag;
}

