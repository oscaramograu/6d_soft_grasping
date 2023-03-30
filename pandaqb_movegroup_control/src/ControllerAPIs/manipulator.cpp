#include <pandaqb_movegroup_control/ControllerAPIs/manipulator.h>
#include "cmath"

Manipulator::Manipulator(): z_offset(0.25){
}

Manipulator::~Manipulator(){
}

void Manipulator::setGraspParams(float theta_, float w_, std::vector <float> graspPoint_){
    graspPoint.position.x = graspPoint_.at(0);
    graspPoint.position.y = graspPoint_.at(1);
    graspPoint.position.z = graspPoint_.at(2);

    theta = theta_;
    w = w_;
};

void Manipulator::setGraspParams(float theta_, float w_, geometry_msgs::Point graspPoint_){
    graspPoint.position = graspPoint_;
    theta = theta_;
    w = w_;
};

void Manipulator::approach(){
    setTargetPosition();
    setTragetOrientation();

    ROS_INFO_STREAM("Moving to target pose.");
    visual_tools->publishAxisLabeled(graspPoint, "grasp_pose");
    moveArmToPose(targetPose);
}

void Manipulator::setTargetPosition(){
    targetPose.position = graspPoint.position;
    targetPose.position.z += z_offset;
}

void Manipulator::setTragetOrientation(){
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