#include <pandaqb_movegroup_control/CameraAPIs/camera.h>

Camera::Camera(){
    setFakeGraspParams();
    RequestGraspServer = nh_.advertiseService("/RequestGrasp", &Camera::RequestGraspCallback, this);
    ROS_INFO_STREAM("The Service /RequestGrasp is READY");
}

Camera::~Camera(){
}

bool Camera::RequestGraspCallback(pandaqb_movegroup_control::RequestGrasp::Request &req,
        pandaqb_movegroup_control::RequestGrasp::Response &resp){
    resp.theta = theta; 
    resp.w = w;
    resp.pose = graspPoint;
    return true;
}

void Camera::setFakeGraspParams(){
    std::vector <float> graspPointVect;

    ros::param::get("fake_params/grasp_position", graspPointVect);
    ros::param::get("fake_params/theta", theta);
    ros::param::get("fake_params/w", w);

    graspPoint.x = graspPointVect.at(0);
    graspPoint.y = graspPointVect.at(1);
    graspPoint.z = graspPointVect.at(2);
}