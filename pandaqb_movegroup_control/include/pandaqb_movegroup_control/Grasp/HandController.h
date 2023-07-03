#include <pandaqb_movegroup_control/Grasp/HandControllerBase.h>

class GraspExecuter: public HandControllerBase{
public:
    GraspExecuter();
    ~GraspExecuter();

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
    std::vector<float> motor_pinch;
    std::vector<float> motor_power;
    std::vector<float> motor_open;

    bool open_flag;
    float duration;
};    