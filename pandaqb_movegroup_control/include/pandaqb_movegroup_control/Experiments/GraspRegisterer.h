#include <ros/ros.h>
#include <iostream>
#include <fstream>

#include <std_srvs/SetBool.h>
#include <std_msgs/String.h>

#include <pandaqb_movegroup_control/Target/GraspListener.h>

class GraspRegisterer: public GraspListener{
public:
    GraspRegisterer(ros::NodeHandle *nh);
    ~GraspRegisterer();

    void register_data();

private:
    void set_file_path();
    void open_file();
    
    void set_data();

    void set_grasp();
    void send_grasp_request();
    void set_success();

    std::ofstream *file;
    std::string exp_files_path, file_path;

    ros::ServiceClient grasp_client;
    std_srvs::SetBool srv;

    ros::Publisher pub;
    std_msgs::String msg;

    int id;
    bool success;
    float width;
    std::string grasp_type, object, eef;
};