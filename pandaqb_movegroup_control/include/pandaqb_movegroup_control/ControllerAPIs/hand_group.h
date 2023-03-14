#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>

class HandGroup{
public:
    HandGroup();
    ~HandGroup();

    void grasp(double synergy_joint);
    void printCurrentJointPosition();


protected:
    void moveJointSpace(double synergy_joint);

private:
    void PlanExecute();
    
    std::string PLANNING_GROUP;

    // Move Group attributes
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    moveit::planning_interface::MoveGroupInterface::Plan plan;
 };