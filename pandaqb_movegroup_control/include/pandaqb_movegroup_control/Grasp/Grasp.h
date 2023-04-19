#include <moveit/planning_scene_interface/planning_scene_interface.h>

class Grasp{
public:
    float theta, w;
    geometry_msgs::Pose::_position_type point;
};