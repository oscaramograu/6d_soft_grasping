#include <pandaqb_movegroup_control/Grasp/GraspExecuter.h>  

GraspExecuter::GraspExecuter():
    Hand("qb_hand"), Arm("panda_arm"){
    closed_hand = {0.3};
    opened_hand = {0.0};
}

GraspExecuter::~GraspExecuter(){
}

void GraspExecuter::grasp(Grasp TargetGrasp){
    geometry_msgs::Pose graspingPose;

    graspingPose = graspToPose(TargetGrasp);

    Arm.moveTo(graspingPose);
    Hand.moveTo(closed_hand);
}

geometry_msgs::Pose GraspExecuter::graspToPose(Grasp TargetGrasp){
    geometry_msgs::Pose graspingPose;


}