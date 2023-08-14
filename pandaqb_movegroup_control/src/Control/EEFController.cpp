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

void EEFController::close(){
    if(using_qb){
        bool pwr_gr_flag;
        pwr_gr_flag = get_power_gr_flag();
        hc->grasp(pwr_gr_flag);
    }
    else{
        float width;
        width = get_width();
        ROS_INFO_STREAM("THE WIDTH IS" << width);
        gc->close_gripper(width);
    }
}

void EEFController::open(){
    if(using_qb){
        hc->open();
    }
    else{
        gc->open_gripper();
    }
}
