#include <pandaqb_movegroup_control/MoveGroup/GroupMover.h>

GroupMover::GroupMover(std::string planning_group): InfoTriggerMoveGroup(planning_group){
}

GroupMover::~GroupMover(){
}

// Cartesianspace
void GroupMover::moveTo(const geometry_msgs::Pose& pose){
    ROS_INFO_STREAM("Moving to cartesian space.");

    move_group_->setPoseTarget(pose);

    vizPlan(pose);
    planExecute();
    vizEnd();
} 

// Jointspace
void GroupMover::moveTo(std::vector<double> joints){
    ROS_INFO_STREAM("Moving to joint space.");

    move_group_->setJointValueTarget(joints);

    planExecute();
}

void GroupMover::moveHome(){
    ROS_INFO_STREAM("Start homing.");

    vizHoming();
    moveTo(home);
    vizEnd();

    ROS_INFO_STREAM("Robot set to home pose.");
}

void GroupMover::set_EEF_link(std::string arm_eef_frame){
    move_group_->setEndEffectorLink(arm_eef_frame);
    ROS_INFO_STREAM("The end effector frame is: " 
    << move_group_->getEndEffectorLink());
}

void GroupMover::planExecute(){
    bool success = (move_group_->plan(plan)==moveit::core::MoveItErrorCode::SUCCESS);

    if (success){
        ROS_INFO_STREAM("Planning succeeded");
        ROS_INFO_STREAM("Press keyboard to execute it");
        std::string user_input;
        std::getline(std::cin, user_input);
        move_group_->execute(plan);
        ROS_INFO_STREAM("Plan executed.");
    }
    else{
        ROS_WARN("Planning failed");
    }
}