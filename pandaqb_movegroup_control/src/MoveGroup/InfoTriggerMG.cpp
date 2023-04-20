#include <pandaqb_movegroup_control/MoveGroup/InfoTriggerMG.h>

InfoTriggerMoveGroup::InfoTriggerMoveGroup(std::string planning_group):

// CHECK THIS OUT !!
    BaseMoveGroup(planning_group){
    visual_tools = std::make_shared<moveit_visual_tools::MoveItVisualTools> ("panda_link0");
    visual_tools->deleteAllMarkers();

    text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = 1.0;
    visual_tools->publishText(text_pose, "Robot controller started", rvt::WHITE, rvt::XLARGE);
    visual_tools->trigger();

    ROS_INFO_NAMED("tutorial", "Planning frame: %s", move_group_->getPlanningFrame().c_str());
    ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group_->getEndEffectorLink().c_str());

    ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
    std::copy(move_group_->getJointModelGroupNames().begin(),
        move_group_->getJointModelGroupNames().end(), 
        std::ostream_iterator<std::string>(std::cout, ", "));
}

InfoTriggerMoveGroup::~InfoTriggerMoveGroup(){
}

geometry_msgs::Pose InfoTriggerMoveGroup::getCurrentPose(){
    return move_group_->getCurrentPose().pose;
}

std::vector<double> InfoTriggerMoveGroup::getCurrentJointState(){
    moveit::core::RobotStatePtr current_state;
    const moveit::core::JointModelGroup* joint_model_group_;
    std::vector<double> joint_group_positions;

    // Safe the current robot states (with velocities and accelerations)
    current_state = move_group_->getCurrentState();
    joint_model_group_ = current_state->getJointModelGroup(PLANNING_GROUP);

    // Get the current joint values of the group
    current_state->copyJointGroupPositions(joint_model_group_, joint_group_positions);

    return joint_group_positions;
}

void InfoTriggerMoveGroup::printCurrentJointPosition(){
    std::vector<double> joint_group_positions = getCurrentJointState();

    // Turn the values to string
    std::string joint_positions = "[";
    joint_positions.append(std::to_string(joint_group_positions[0]));

    for (int i=1; i<joint_group_positions.size(); i++){
        joint_positions.append(", ");
        joint_positions.append(std::to_string(joint_group_positions[i]));
    }
    joint_positions.append("]");

    ROS_INFO_STREAM("Joint values: " << joint_positions);
}

void InfoTriggerMoveGroup::vizPlan(geometry_msgs::Pose target_pose){
    visual_tools->publishText(text_pose, "Moving to target pose", rvt::WHITE, rvt::XLARGE);
    visual_tools->publishAxisLabeled(target_pose, "target_pose");
    visual_tools->trigger();
}

void InfoTriggerMoveGroup::vizEnd(){
    visual_tools->deleteAllMarkers();
    visual_tools->trigger();
}

void InfoTriggerMoveGroup::vizHoming(){
    visual_tools->publishText(text_pose, "Homing", rvt::WHITE, rvt::XLARGE);
    visual_tools->trigger();
}
