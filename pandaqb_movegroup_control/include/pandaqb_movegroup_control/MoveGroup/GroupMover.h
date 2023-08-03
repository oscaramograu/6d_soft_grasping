#ifndef GROUPMOVER
#define GROUPMOVER

#include <pandaqb_movegroup_control/MoveGroup/InfoTriggerMG.h>
#include <string>   // for string manipulation

class GroupMover : public InfoTriggerMoveGroup{
public:
    GroupMover(std::string planning_group);
    virtual ~GroupMover();

    void moveTo(const geometry_msgs::Pose& pose); // Cartesianspace
    void moveTo(std::vector<double> joints); // Jointspace

    void moveHome();

    void set_EEF_link(std::string arm_eef_frame);

    void clear_waypt();
    void apend_waypt(geometry_msgs::Pose pose);
    void build_cart_plan();
    
private:
    void planExecute();
};
#endif // GROUPMOVER