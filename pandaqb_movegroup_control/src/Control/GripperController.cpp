#include <pandaqb_movegroup_control/Control/GripperController.h>

GripperController::GripperController(ros::NodeHandle *nh):
        grasp_ac("/franka_gripper/grasp", true), move_ac("/franka_gripper/move"){
    move_goal.speed = 1;
    move_goal.width = 0.08;
}

GripperController::~GripperController(){
}

void GripperController::close_gripper(float width){
    ROS_INFO_STREAM("STARTING GRASPING PROCESS");
    build_grasp(width);
    send_grasp_goal();
}

void GripperController::open_gripper(){
    ROS_INFO_STREAM("STARTING OPENING PROCESS");
    send_move_goal();
}

void GripperController::build_grasp(float width){
    grasp_goal.width = width;
    grasp_goal.epsilon.inner = 0.01;
    grasp_goal.epsilon.outer = 0.01;
    grasp_goal.speed = 0.1;
    grasp_goal.force = 5.0;
}

void GripperController::send_grasp_goal(){
    grasp_ac.sendGoal(grasp_goal);
    bool finished_before_timeout = grasp_ac.waitForResult(ros::Duration(10.0));
    if (finished_before_timeout)
    {
    actionlib::SimpleClientGoalState state = grasp_ac.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
    }
    else
    {
    ROS_INFO("Action did not finish before the time out.");
    grasp_ac.cancelGoal();
    }
}

void GripperController::send_move_goal(){
    move_ac.sendGoal(move_goal);
    bool finished_before_timeout = move_ac.waitForResult(ros::Duration(10.0));
    if (finished_before_timeout)
    {
    actionlib::SimpleClientGoalState state = move_ac.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
    }
    else
    {
    ROS_INFO("Action did not finish before the time out.");
    move_ac.cancelGoal();
    }
}