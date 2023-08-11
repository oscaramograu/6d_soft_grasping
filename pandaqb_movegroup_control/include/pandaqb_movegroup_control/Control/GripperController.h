#include <pandaqb_movegroup_control/MoveGroup/GroupMover.h>

class GripperController: public GroupMover{
public:
    GripperController();
    ~GripperController();

    void close_gripper(float width);
    void open_gripper();

private:
    std::vector<double> open_with{0.04, 0.04};
};