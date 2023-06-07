#include <ros/ros.h>
#include <Eigen/Dense>
#include <vector>
#include <std_msgs/Float32MultiArray.h>

typedef Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> MatMapType;
typedef std_msgs::Float32MultiArray::ConstPtr MultyArrayPtr;

class ObjWorld_TPublisher{
public:
  ObjWorld_TPublisher();
  ~ObjWorld_TPublisher();

  void callback(const MultyArrayPtr &ObjCam_T_msg);

private:
  ros::NodeHandle nh_;
  ros::Publisher pub_;
  ros::Subscriber sub_;

  std_msgs::MultiArrayLayout::Ptr msg_layout;

  void set_msg_layout(const MultyArrayPtr msg);

  Eigen::MatrixXd MultiArray_to_Eigen(const MultyArrayPtr& arr_msg);
  MultyArrayPtr Eigen_to_MultiArray(Eigen::MatrixXd eigen__arr);

  Eigen::MatrixXd compute_ObjWorld(Eigen::MatrixXd ObjCam_T);
};