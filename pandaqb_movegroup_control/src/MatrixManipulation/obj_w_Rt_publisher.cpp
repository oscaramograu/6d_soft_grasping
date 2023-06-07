#include <pandaqb_movegroup_control/MatrixManipulation/obj_w_Rt_publisher.h>

ObjWorld_TPublisher::ObjWorld_TPublisher(){
    // Initialize ROS publisher and subscriber
    pub_ = nh_.advertise<std_msgs::Float32MultiArray>("obj_worldRt_topic", 10);
    sub_ = nh_.subscribe("/obj_to_cam_Rt", 10, &ObjWorld_TPublisher::callback, this);
}

ObjWorld_TPublisher::~ObjWorld_TPublisher(){
}

void ObjWorld_TPublisher::callback(const MultyArrayPtr &ObjCam_T_msg)
{
    set_msg_layout(ObjCam_T_msg);

    Eigen::MatrixXd ObjCam_T = MultiArray_to_Eigen(ObjCam_T_msg);
    Eigen::MatrixXd ObjWorld_T = compute_ObjWorld(ObjCam_T);
    const MultyArrayPtr ObjWorld_T_msg = Eigen_to_MultiArray(ObjWorld_T);

    pub_.publish(*ObjWorld_T_msg);    

    ROS_INFO_STREAM("The matrix recieved is: \n" << ObjCam_T);
    ROS_INFO_STREAM("The matrix published is: \n" << ObjWorld_T);
}

void ObjWorld_TPublisher::set_msg_layout(const MultyArrayPtr msg){
    *msg_layout = msg->layout; 
}

Eigen::MatrixXd ObjWorld_TPublisher::MultiArray_to_Eigen(const MultyArrayPtr &arr_msg) {
    const int row = msg_layout->dim[0].size;
    const int col = msg_layout->dim[1].size;

    const float* data_vec = arr_msg->data.data();

    Eigen::MatrixXd eigen_arr = Eigen::Map<const MatMapType>(data_vec, row, col).cast<double>();

    return eigen_arr;
}

MultyArrayPtr ObjWorld_TPublisher::Eigen_to_MultiArray(Eigen::MatrixXd eigen_arr){
    std_msgs::Float32MultiArray::Ptr arr_msg;
    arr_msg->layout = *msg_layout;

    std::vector<float> stdVector(eigen_arr.data(), eigen_arr.data() + eigen_arr.size());
    
    arr_msg->data = stdVector;
    return arr_msg;
}

Eigen::MatrixXd ObjWorld_TPublisher::compute_ObjWorld(Eigen::MatrixXd ObjCam_T){
    // TO DO
    Eigen::MatrixXd ObjWorld_T;
    return ObjWorld_T;
}