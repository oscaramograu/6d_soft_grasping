#include <moveit_visual_tools/moveit_visual_tools.h>
#include <pandaqb_movegroup_control/MoveGroup/BaseMoveGroup.h>

namespace rvt = rviz_visual_tools;

class InfoTriggerMoveGroup: BaseMoveGroup{
public:
    InfoTriggerMoveGroup();
    virtual ~InfoTriggerMoveGroup();

    geometry_msgs::Pose getCurrentPose();
    geometry_msgs::Pose::_orientation_type getCurrentOrientation();

    void printCurrentJointPosition();

protected:
    void visualizePlan(const moveit::planning_interface::MoveGroupInterface::Plan& plan);

private:
    std::shared_ptr<moveit_visual_tools::MoveItVisualTools> visual_tools_;
    Eigen::Isometry3d text_pose_;
};
