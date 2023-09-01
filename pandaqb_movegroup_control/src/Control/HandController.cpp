#include <pandaqb_movegroup_control/Control/HandController.h>

HandController::HandController(ros::NodeHandle *nh){
    traj_msg.joint_names = {"qbhand2m1_synergy_joint", "qbhand2m1_manipulation_joint"};
    init_point();

    traj_syn_pub = nh->advertise<trajectory_msgs::JointTrajectory>(
        "/qbhand2m1/control/qbhand2m1_synergies_trajectory_controller/command", 1);
}

HandController::~HandController(){
}

void HandController::grasp(std::vector<double> synergies){
    ROS_INFO_STREAM("New plan: HAND GRASP");
    std::vector<double> hand_joints = {synergies[1], synergies[0]};
    publish_traj(synergies);
}

void HandController::open(){
    std::vector<double> open_syn{0, 0};
    ROS_INFO_STREAM("New plan: HAND OPEN");
    publish_traj(open_syn);
}

void HandController::publish_traj(std::vector<double> synergies){
    point.positions = synergies;

    traj_msg.points = {point};
    ROS_INFO_STREAM("The synergies are: " << synergies[0] << ", " << synergies[1]);

    ROS_INFO_STREAM("Press enter key to execute it");
    std::string user_input;
    std::getline(std::cin, user_input);

    traj_syn_pub.publish(traj_msg);
}

void HandController::init_point(){
    point.velocities.push_back(0.0);
    point.velocities.push_back(0.0);
    point.accelerations.push_back(0.0);
    point.accelerations.push_back(0.0);
    point.effort.push_back(0.0);
    point.effort.push_back(0.0);
    point.time_from_start.sec = 1;
}