#include <pandaqb_movegroup_control/ControllerAPIs/target_aproacher.h>
#include "cmath"

TargetApproacher::TargetApproacher(): z_offset(0.25)
{
    moveToHomePose();
    graspingClient = nh_.serviceClient<pandaqb_movegroup_control::GraspingService>("GraspingService");
    image_path = "/home/oscar/catkin_ws/src/Thesis/ImageData/first_image.png";
}

TargetApproacher::~TargetApproacher(){
}

// Call request to get grasp pose, turn to target pose and move there
void TargetApproacher::approach(){
    ROS_INFO_STREAM("Grasp target requested.");
    requestGraspPose();

    getTargetPose();
    getTragetOrientation();

    ROS_INFO_STREAM("Moving to target pose.");
    moveArmToPose(targetPose);
}

// Make a request to the DL model to get the grasp from an image
void TargetApproacher::requestGraspPose(){
    ih.set_ros_image(image_path);
    srv.request.image = ih.get_ros_image();

    if (graspingClient.call(srv))
    {
        ROS_INFO_STREAM("Parameters recieved from GraspingServer");
        theta = srv.response.theta;
        w = srv.response.w;
        graspPose = srv.response.pose;
    }
    else
    {
        ROS_ERROR("Failed to call service GraspingService");
    } 
}

// Get target pose position
void TargetApproacher::getTargetPose(){
    targetPose.position = graspPose;
    targetPose.position.z += z_offset;
}

// Get target pose orientation based on the obtained theta
void TargetApproacher::getTragetOrientation(){
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