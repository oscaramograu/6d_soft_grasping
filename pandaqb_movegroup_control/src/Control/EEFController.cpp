#include <pandaqb_movegroup_control/Control/EEFController.h>

EEFController::EEFController(ros::NodeHandle *nh): GraspListener(nh),
        hc(nullptr), gc(nullptr){
    std::string rbt_conf;
    ros::param::get("robot_config", rbt_conf);

    if(rbt_conf == "qb_hand"){
        using_qb = true;
        hc = new HandController();
    }
    else if(rbt_conf == "gripper"){
        using_qb = false;
        gc = new GripperController();
    }
    else{
        ROS_INFO_STREAM("Wrong eef in yaml file.");
    }
}   

EEFController::~EEFController(){
    delete gc;
    delete hc;
}

void EEFController::grasp(){
    if(using_qb){
        std::vector<double> sinergies = get_sinergies();
        // sinergies = {0.4, 0.1};
        hc->grasp(sinergies);
    }
    else{
        float width;
        width = get_width();
        gc->close_gripper(width);
    }
}

void EEFController::close_hand(){
    std::vector<double> power_sinergy = {0.9, 0};
    hc->grasp(power_sinergy);
}

void EEFController::open(){
    if(using_qb){
        hc->open();
    }
    else{
        gc->open_gripper();
    }
}
