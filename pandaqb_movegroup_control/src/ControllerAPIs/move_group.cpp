#include <pandaqb_movegroup_control/ControllerAPIs/MoveIt/move_group.h>

// Coonstructor for the PandaAPI class
MoveGroup::MoveGroup(): PLANNING_GROUP("arm")
{
    // Instatiate the move_group 
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(PLANNING_GROUP);

    // Load values from ros param server and setup  the move_group interface
    LoadParams();  

    // Initialize moveit visual tools
    InitVisualTools();
}

MoveGroup::~MoveGroup()
{   
}

// Move the arm to a specified pose
void MoveGroup::moveArmToPose(const geometry_msgs::Pose& pose){
    move_group_->setPoseTarget(pose);
    PlanVisualize(pose);
    // Plan and execute the motion plan
    PlanExecute();
    visual_tools->deleteAllMarkers();
};

// Move the arm to a set of waypoints in cartesian space
void MoveGroup::cartesianSpaceMotion(std::vector<geometry_msgs::Pose>& waypoints){
    // plan
    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = move_group_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    ROS_INFO_NAMED("Manipulator Control", "Cartesian path (%.2f%% acheived)", fraction * 100.0);
    if (fraction < 1)
    {
        ROS_ERROR_STREAM("Cartesian path not 100% acheived");
    }

    // execute
    move_group_->execute(trajectory);
}

// Move the arm to a desired joint state position
void MoveGroup::moveJointSpace(std::vector<double> abs_pos){
    // Check input validity
    if(abs_pos.size() != 7){
        ROS_ERROR_STREAM("PandaAPI::moveJointSpace(): input dimensions not valid!");
        ROS_ERROR_STREAM("Current dimensions are: " << abs_pos.size());
    }

    // Set target value
    move_group_->setJointValueTarget(abs_pos);

    // Plan
    PlanExecute();
}

void MoveGroup::printCurrentJointPosition(){
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

// Move the arm to the home position
void MoveGroup::moveToHomePose(){
    ROS_INFO_STREAM("Start homing.");
    moveJointSpace(home);
    ROS_INFO_STREAM("Robot set to home pose.");
}

// Load values from ros param server and setup the move_group interface
void MoveGroup::LoadParams()
{
    ros::param::get("move_group_params/home", home);
    ros::param::get("move_group_params/planning_time", planning_time);
    ros::param::get("move_group_params/planning_attempts", planning_attempts);
    ros::param::get("move_group_params/velocity_scaling", velocity_scaling);
    ros::param::get("move_group_params/acceleration_scaling", acceleration_scaling);

    // Move group setup
    move_group_->setPlanningTime(planning_time);
    move_group_->setNumPlanningAttempts(planning_attempts);
    move_group_->setMaxVelocityScalingFactor(velocity_scaling);
    move_group_->setMaxAccelerationScalingFactor(acceleration_scaling);
}

// Initialize Moveit visual tools and print some initial msgs
void MoveGroup::InitVisualTools(){
    visual_tools = std::make_shared<moveit_visual_tools::MoveItVisualTools> ("panda_link0");
    visual_tools->deleteAllMarkers();
    visual_tools->trigger();


    text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = 1.0;
    visual_tools->publishText(text_pose, "Robot controller started", rvt::WHITE, rvt::XLARGE);
    

    ROS_INFO_NAMED("tutorial", "Planning frame: %s", move_group_->getPlanningFrame().c_str());
    ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group_->getEndEffectorLink().c_str());

    ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
    std::copy(move_group_->getJointModelGroupNames().begin(),
        move_group_->getJointModelGroupNames().end(), 
        std::ostream_iterator<std::string>(std::cout, ", "));
}

// Visualize the plan
void MoveGroup::PlanVisualize(geometry_msgs::Pose target_pose){
    ROS_INFO_NAMED("Robot controller", "Visualizing plan as trajectory line");
    visual_tools->publishAxisLabeled(target_pose, "target");
    visual_tools->publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
    visual_tools->publishTrajectoryLine(plan.trajectory_, move_group_->getCurrentState()->getJointModelGroup(PLANNING_GROUP));
    visual_tools->trigger();
}

// Plan and Execute
void MoveGroup::PlanExecute(){
    bool success = (move_group_->plan(plan)==moveit::core::MoveItErrorCode::SUCCESS);

    if (success){
        ROS_INFO_STREAM("Planning succeeded");
        // visual_tools->publishTrajectoryLine(plan.trajectory_, move_group_->getCurrentState()->getJointModelGroup(PLANNING_GROUP));

        move_group_->execute(plan);
        ROS_INFO_STREAM("Plan executed.");
    }
    else{
        ROS_WARN("Planning failed");
    }
}

geometry_msgs::Pose MoveGroup::getCurrentPose(){
    return move_group_->getCurrentPose().pose;
}

geometry_msgs::Pose::_orientation_type MoveGroup::getCurrentOrientation(){
    return getCurrentPose().orientation;
}