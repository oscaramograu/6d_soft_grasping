#include <pandaqb_movegroup_control/Control/GripperController.h>
#include <pandaqb_movegroup_control/Control/HandController.h>
#include <pandaqb_movegroup_control/Target/GraspListener.h>

class EEFController: public GraspListener{
public:
    EEFController(ros::NodeHandle *nh);
    ~EEFController();

    void open();
    void grasp();
    void close_hand();

private:
    GripperController* gc;
    HandController* hc;

    bool using_qb;
};