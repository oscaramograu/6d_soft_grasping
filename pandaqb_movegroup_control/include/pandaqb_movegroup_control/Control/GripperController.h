#include <franka_gripper/GraspAction.h>
#include <franka_gripper/MoveAction.h>
#include <actionlib/client/simple_action_client.h>

class GripperController{
public:
    GripperController(ros::NodeHandle *nh);
    ~GripperController();

    void close_gripper(float width);
    void open_gripper();

private:
    void build_grasp(float widht);
    void send_grasp_goal();
    void send_move_goal();

    actionlib::SimpleActionClient<franka_gripper::GraspAction> grasp_ac;
    actionlib::SimpleActionClient<franka_gripper::MoveAction> move_ac;

    franka_gripper::GraspGoal grasp_goal;
    franka_gripper::MoveGoal move_goal;
};