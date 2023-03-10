#include <pandaqb_movegroup_control/ControllerAPIs/grasper.h>
#include "cmath"

TargetApproacher::TargetApproacher(float theta_, float w_, std::vector <float> GraspPoint_):
    z_offset(0.25), theta(theta_), w(w_)
{
    graspPoint.position.x = GraspPoint_.at(0);
    graspPoint.position.y = GraspPoint_.at(1);
    graspPoint.position.z = GraspPoint_.at(2);
}

TargetApproacher::~TargetApproacher(){
}

// Call request to get grasp pose, turn to target pose and move there
void TargetApproacher::approach(){
    setTargetPosition();
    setTragetOrientation();

    ROS_INFO_STREAM("Moving to target pose.");
    visual_tools->publishAxisLabeled(graspPoint, "grasp_pose");
    moveArmToPose(targetPose);
}

// Get target pose position
void TargetApproacher::setTargetPosition(){
    targetPose.position = graspPoint.position;
    targetPose.position.z += z_offset;
}

// Get target pose orientation based on the obtained theta
void TargetApproacher::setTragetOrientation(){
    // Rotation arorund Z from theta obtained
    Eigen::Matrix3d Rz;
    Rz << cos(theta), -sin(theta), 0,
           sin(theta), cos(theta), 0,
           0, 0, 1;
    Eigen::Quaterniond Qz{Rz};

    // Current orientation
    geometry_msgs::Pose::_orientation_type CurrentOrient;
    CurrentOrient = getCurrentOrientation();
    Eigen::Quaterniond CurrentQ, NewQ;
    CurrentQ = {
        CurrentOrient.w,
        CurrentOrient.x,
        CurrentOrient.y,
        CurrentOrient.z
    };

    // Apply the rotation arround Z to the current orientation to get the new one
    NewQ = Qz*CurrentQ;
    targetPose.orientation.x = NewQ.coeffs()(0);
    targetPose.orientation.y = NewQ.coeffs()(1);
    targetPose.orientation.z = NewQ.coeffs()(2);
    targetPose.orientation.w = NewQ.coeffs()(3);
}