#include <pandaqb_movegroup_control/Control/Controller.h>

Controller::Controller(ros::NodeHandle *nh): GraspListener(nh),
        arm_controller(), eef_controller(nh){
}

Controller::~Controller(){
}

void Controller::start_new_routine(){
    std::cout << "PRESS ENTER TO START A NEW ROUTINE: ";
    target_pose = get_grasp_pose();
    place_pose = get_place_pose();
    arm_controller.set_grasp(target_pose);
    arm_controller.set_place_pose(place_pose);

    pick_and_place_routine();
}

void Controller::pick_and_place_routine(){
    eef_controller.open();
    arm_controller.approach_grasp();

    arm_controller.move_to_g_pose();

    eef_controller.grasp();
    // arm_controller.pick_up();

    // arm_controller.move_to_place_pose();
    // eef_controller.open();

    arm_controller.move_home();
    // eef_controller.close_hand();
    eef_controller.open();
}