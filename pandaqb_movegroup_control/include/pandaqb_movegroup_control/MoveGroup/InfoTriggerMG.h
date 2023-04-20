#include <moveit_visual_tools/moveit_visual_tools.h>
#include <pandaqb_movegroup_control/MoveGroup/BaseMoveGroup.h>

namespace rvt = rviz_visual_tools;

class InfoTriggerMoveGroup: public BaseMoveGroup{
public:
    InfoTriggerMoveGroup(std::string planning_group);
    ~InfoTriggerMoveGroup();

    geometry_msgs::Pose getCurrentPose();
    std::vector<double> getCurrentJointState();
    void printCurrentJointPosition();

protected:
    void vizPlan(geometry_msgs::Pose target_pose);
    void vizHoming();
    void vizEnd();
    
private:
    std::shared_ptr<moveit_visual_tools::MoveItVisualTools> visual_tools;
    Eigen::Isometry3d text_pose;
};
