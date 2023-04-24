#include <pandaqb_movegroup_control/MoveGroup/GroupMover.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

class  RobotMover : public GroupMover{
public:
    RobotMover();
    ~RobotMover();

    void move(geometry_msgs::Pose target);
    
private:
    geometry_msgs::Pose target_to_EEF(geometry_msgs::Pose targetGrasp);
    void pandaLink_to_EEF();

    std::string eef_frame;
};