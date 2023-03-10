#include <pandaqb_movegroup_control/CameraAPIs/camera.h>

Camera::Camera(){
    setFakeGraspParams();
    RequestGraspServer = nh_.advertiseService("/RequestGrasp", &Camera::RequestGraspCallback, this);
    ROS_INFO_STREAM("The Service /RequestGrasp is ready!");

    DetectClient = nh_.serviceClient<pandaqb_movegroup_control::DetectGrasp>("/DetectGrasp");
}

Camera::~Camera(){
}

bool Camera::RequestGraspCallback(pandaqb_movegroup_control::RequestGrasp::Request &req,
        pandaqb_movegroup_control::RequestGrasp::Response &resp){
    ROS_INFO_STREAM("Grasp requested to the Camera Node from the Controller Node");

    ih.set_ros_image(image_path);
    DetectGraspSrv.request.image = ih.get_ros_image();

    if(DetectClient.call(DetectGraspSrv)){
        resp.point = DetectGraspSrv.response.point;
        resp.theta = DetectGraspSrv.response.theta;
        resp.w = DetectGraspSrv.response.w;
    }
    else{
        ROS_ERROR_STREAM("There was an error calling the DetectGrasp service.");
    }
    return true;
}

void Camera::setFakeGraspParams(){
    std::vector <float> graspPointVect;

    ros::param::get("fake_params/grasp_position", graspPointVect);
    ros::param::get("fake_params/theta", theta);
    ros::param::get("fake_params/w", w);
    ros::param::get("fake_params/image_path", image_path);

    graspPoint.x = graspPointVect.at(0);
    graspPoint.y = graspPointVect.at(1);
    graspPoint.z = graspPointVect.at(2);
}