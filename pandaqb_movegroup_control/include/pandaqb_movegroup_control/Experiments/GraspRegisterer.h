#include <ros/ros.h>
#include <iostream>
#include <fstream>

#include <std_srvs/SetBool.h>
#include <std_msgs/String.h>

#include <pandaqb_movegroup_control/Target/GraspListener.h>
#include <pandaqb_movegroup_control/Control/Controller.h>

class GraspRegisterer: public Controller{
public:
    GraspRegisterer(ros::NodeHandle *nh);
    ~GraspRegisterer();

    void register_data();

private:
    void set_file_path();
    void open_file();
    
    void set_data();

    void send_grasp_();
    void set_success(bool &success_var);

    std::ofstream *file;
    std::string exp_files_path, file_path;

    ros::ServiceClient grasp_client;
    std_srvs::SetBool srv;

    ros::Publisher pub;
    std_msgs::String msg;

    int id;
    float width;
    std::vector<double> sinergies;
    bool grasp_success, place_success;

    std::string object, eef;
};