#include <pandaqb_movegroup_control/ControllerAPIs/controller.h>

Controller::Controller(){
    RequestClient = nh_.serviceClient<pandaqb_movegroup_control::RequestGrasp>("/RequestGrasp");
}

Controller::~Controller(){
}

void Controller::routine(){
    // 1 - Move Home
    manipulator.moveToHomePose();

    // 2 - Make a grasp request to the camera node and set the grasp parameters
    RequestGrasp();

    // 3 - Approach target pose
    manipulator.approach();

    // 4 - Grasp the object
    // manipulator.grasp();
}

void Controller::RequestGrasp(){
    if(RequestClient.call(RequestGraspSrv)){
        ROS_INFO_STREAM("Grasp requested to the Camera Node from the Controller Node");
        manipulator.setGraspParams(RequestGraspSrv.response.theta, 
            RequestGraspSrv.response.w, RequestGraspSrv.response.point);
    }
    else{
        ROS_ERROR_STREAM("There was an error calling the RequestGrasp service.");
    }
}