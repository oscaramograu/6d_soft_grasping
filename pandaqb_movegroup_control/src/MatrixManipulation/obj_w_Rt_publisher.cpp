#include <pandaqb_movegroup_control/MatrixManipulation/obj_w_Rt_publisher.h>

ObjWorld_RtPublisher::ObjWorld_RtPublisher(){
    // Initialize ROS publisher and subscriber
    // pub_ = nh_.advertise<std_msgs::Float32MultiArray>("obj_worldRt_topic", 10);
    sub_ = nh_.subscribe("/obj_to_cam_Rt", 10, &ObjWorld_RtPublisher::callback, this);
}

ObjWorld_RtPublisher::~ObjWorld_RtPublisher(){
}


void ObjWorld_RtPublisher::callback(const std_msgs::Float32MultiArray::ConstPtr& obj_cam_Rt_msg)
{
  // const float w = obj_cam_Rt_msg->layout.dim[1].size;
  Eigen::MatrixXd obj_cam_Rt = MultiArray_to_Eigen(obj_cam_Rt_msg);

  ROS_INFO_STREAM("The matrix recieved is: \n" << obj_cam_Rt);

  // ROS_INFO_STREAM("The height of the matrix is: " << h);
  // ROS_INFO_STREAM("The end effector frame is: " << w );


  // Below are a few basic Eigen demos:
  // std::vector<float> data = obj_cam_Rt_msg->data;
  // typedef Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> MatrixType;
  // Eigen::Map<MatrixType> mat(data.data(), h, w);

  // std::cout << "The recieved matrx is: " << std::endl << mat << std::endl;
}

Eigen::MatrixXd ObjWorld_RtPublisher::MultiArray_to_Eigen(const std_msgs::Float32MultiArray::ConstPtr& array_msg) {
    const int row = array_msg->layout.dim[0].size;
    const int col = array_msg->layout.dim[1].size;

    const float* data_vec = array_msg->data.data();

    Eigen::MatrixXd eigen_array = Eigen::Map<MatMapType>(data_vec, row, col).cast<double>();

    return eigen_array;
}



    // Eigen::MatrixXd obj_cam_Rt(data.size(), 1);
    // for (size_t i = 0; i < data.size(); ++i)
    //   obj_cam_Rt(i) = data[i];

    // Eigen::MatrixXd anotherMatrix;  // Set this to the matrix you want to multiply with obj_camRt

    // Eigen::MatrixXd obj_worldRt = obj_cam_Rt * anotherMatrix;

    // // Create a std_msgs/Float32MultiArray message
    // std_msgs::Float32MultiArray objWorldRt_msg;
    // objWorldRt_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    // objWorldRt_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    // objWorldRt_msg.layout.dim[0].label = "rows";
    // objWorldRt_msg.layout.dim[0].size = objWorldRt.rows();
    // objWorldRt_msg.layout.dim[0].stride = objWorldRt.rows() * objWorldRt.cols();
    // objWorldRt_msg.layout.dim[1].label = "cols";
    // objWorldRt_msg.layout.dim[1].size = objWorldRt.cols();
    // objWorldRt_msg.layout.dim[1].stride = objWorldRt.cols();
    // objWorldRt_msg.data.resize(objWorldRt.size());

    // // Copy the matrix data to the message
    // std::copy(objWorldRt.data(), objWorldRt.data() + objWorldRt.size(), obj_cam_Rt_msg.data.begin());

    // // Publish the message
    // pub_.publish(objWorldRt);