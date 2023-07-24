#include <pandaqb_movegroup_control/MoveGroup/GroupMover.h>

class HandController{
public:
    HandController();
    ~HandController();

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
    /**
     * Opens the qb soft hand.
     */
    void open();
    /**
     * Checks the open flag and retrieves its value.
     * The flag is true if the last method called was a open().
     */
    bool is_open();

private:
    void send_trajectory(std::vector<double> motor_command);

    GroupMover hand_mover;
    bool open_flag;
    float duration;
};    