#include <ros/ros.h>
#include <Eigen/Dense>
#include <vector>
#include <std_msgs/Float32MultiArray.h>

class ObjWorld_RtPublisher{
public:

    ObjWorld_RtPublisher();
    ~ObjWorld_RtPublisher();

    void callback(const std_msgs::Float32MultiArray::ConstPtr &ObjCam_Rt);

private:
  ros::NodeHandle nh_;
  // ros::Publisher pub_;
  ros::Subscriber sub_;

  Eigen::MatrixXd MultiArray_to_Eigen(const std_msgs::Float32MultiArray::ConstPtr& ArrayMsg);
  Eigen::MatrixXd obj_world_mat_converter(Eigen::MatrixXd ObjCam_Rt);

  typedef const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> MatMapType;
};