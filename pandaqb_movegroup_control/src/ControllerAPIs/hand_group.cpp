#include <pandaqb_movegroup_control/ControllerAPIs/hand_group.h>

HandGroup::HandGroup(): PLANNING_GROUP("qb_hand"){
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(PLANNING_GROUP);
}
HandGroup::~HandGroup(){}

void HandGroup::moveJointSpace(double synergy_joint){
    std::vector <double> synergy_joints;
    synergy_joints.push_back(synergy_joint);
    ROS_INFO_STREAM("Set hand group synergy to: " << synergy_joint);

    move_group_->setJointValueTarget(synergy_joints);

    PlanExecute();
}


void HandGroup::grasp(double synergy_joint){
    moveJointSpace(synergy_joint);
    ROS_INFO_STREAM("Grasp Requested");
}

void HandGroup::PlanExecute(){
    bool success = (move_group_->plan(plan)==moveit::core::MoveItErrorCode::SUCCESS);

    if (success){
        ROS_INFO_STREAM("Planning succeeded");
        move_group_->execute(plan);
        ROS_INFO_STREAM("Plan executed.");
    }
    else{
        ROS_WARN("Planning failed");
    }
}

void HandGroup::printCurrentJointPosition(){
    moveit::core::RobotStatePtr current_state;
    const moveit::core::JointModelGroup* joint_model_group_;
    std::vector<double> joint_group_positions;

    // Safe the current robot states (with velocities and accelerations)
    current_state = move_group_->getCurrentState();
    joint_model_group_ = current_state->getJointModelGroup(PLANNING_GROUP);

    // Get the current joint values of the group
    current_state->copyJointGroupPositions(joint_model_group_, joint_group_positions);

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