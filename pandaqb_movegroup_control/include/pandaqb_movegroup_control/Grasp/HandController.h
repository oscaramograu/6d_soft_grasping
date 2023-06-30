#include <pandaqb_movegroup_control/Grasp/HandControllerBase.h>

class GraspExecuter: public HandControllerBase{
public:
    GraspExecuter();
    ~GraspExecuter();

    void pinch();
    void power();
    void open();

    bool is_open();

private:
    std::vector<float> motor_pinch;
    std::vector<float> motor_power;
    std::vector<float> motor_open;

    bool open_flag;
    float duration;
};    