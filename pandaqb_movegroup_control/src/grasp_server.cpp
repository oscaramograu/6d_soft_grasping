#include <pandaqb_movegroup_control/GraspingService.h>
#include <ros/ros.h>
#include <cmath>

std::vector<double> position;
double theta, w;

bool dummy_function(pandaqb_movegroup_control::GraspingService::Request  &req,
         pandaqb_movegroup_control::GraspingService::Response &res)
{

  res.pose.x = position[0];
  res.pose.y = position[1];
  res.pose.z = position[2];

  res.theta = theta*M_PI;
  res.w = w;

  ROS_INFO_STREAM("Parameters are set: theta = " << res.theta << ", w = " << res.w << "\n " << res.pose);

  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "grasp_server");
  ros::NodeHandle nh;

  ros::param::get("dummy_nn_node/grasp_position", position);
  ros::param::get("dummy_nn_node/theta", theta);
  ros::param::get("dummy_nn_node/w", w);

  ros::ServiceServer service = nh.advertiseService("GraspingService", dummy_function);
  ROS_INFO_STREAM("Ready to send Grrasps for an image.");

  ros::spin();

  return 1;
}