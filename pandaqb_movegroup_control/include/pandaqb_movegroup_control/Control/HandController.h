#include <pandaqb_movegroup_control/MoveGroup/GroupMover.h>

class HandController: public GroupMover{
public:
    HandController();
    ~HandController();
    
    void grasp(bool pw_gr_flag);

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

    /**
     * Makes the robot qb soft hand execute a pinch grasp. 
     * Closing only the index and the thumb.
     */
    void pinch();
    /**
     * Makes the robot qb soft hand execute a power grasp.
     * Closing all the fingers.
     */
    void power();

    bool open_flag;
    float duration;
};    