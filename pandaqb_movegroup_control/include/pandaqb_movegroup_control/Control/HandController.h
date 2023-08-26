#include <pandaqb_movegroup_control/MoveGroup/GroupMover.h>

class HandController: public GroupMover{
public:
    HandController();
    ~HandController();
    
    void grasp(std::vector<float> sinergies);

    /**
     * Opens the qb soft hand.
     */
    void open();

    /**
     * Checks the open flag and retrieves its value.
     * The flag is true if the last method called was a open().
     */
    bool is_open();
    
    void print_joints();

private:
    void send_trajectory(std::vector<double> motor_command);

    bool open_flag;
    float duration;
};    