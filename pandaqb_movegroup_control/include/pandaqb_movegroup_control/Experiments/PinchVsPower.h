#pragma once
#include <ros/ros.h>
#include <iostream>
#include <fstream>

#include <std_srvs/SetBool.h>
#include <std_msgs/String.h>

#include <pandaqb_movegroup_control/Target/GraspListener.h>

class PinchVsPower: public GraspListener{
public:
    PinchVsPower(ros::NodeHandle *nh, bool using_clutter);
    ~PinchVsPower();

    void register_data();

private:
    void open_file();

    void set_data();

    void set_grasp();
    void send_grasp_request();
    void set_success();

    std::ofstream *file;
    std::string file_path;

    ros::ServiceClient grasp_client;
    std_srvs::SetBool srv;

    ros::Publisher pub;
    std_msgs::String msg;

    int id;
    bool success, clutter;
    float width;
    std::string grasp, object;
};