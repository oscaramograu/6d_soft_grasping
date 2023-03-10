#include <ros/ros.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

class PlanningScene{
public:
    PlanningScene();
    ~PlanningScene();


private:
    moveit::planning_interface::PlanningSceneInterface planning_scene_;