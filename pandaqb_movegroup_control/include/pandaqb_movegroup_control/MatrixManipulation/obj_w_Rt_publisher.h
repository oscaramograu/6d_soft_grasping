#include <vector>
#include <std_msgs/Float32MultiArray.h>
#include <pandaqb_movegroup_control/MatrixManipulation/ObjWorldTransformer.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>

typedef Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> MatMapType;
typedef std_msgs::Float32MultiArray::Ptr MultyArrayPtr;

class ObjWorld_TPublisher: public ObjWorldTransformer{
public:
  ObjWorld_TPublisher();
  ~ObjWorld_TPublisher();

  void ObjCam_callback(const MultyArrayPtr &ObjCam_T_msg);

private:
  ros::NodeHandle nh_;
  ros::Subscriber ObjCam_sub_;

  Eigen::Affine3d MultiArray_to_Affine(const MultyArrayPtr& arr_msg);
  Eigen::Affine3d get_EEFWorld_T_from_tf();
  
  void broadcast_tf(tf::Transform ObjWorld_tf);
};