#include <pandaqb_movegroup_control/Experiments/PinchVsPower.h>

PinchVsPower::PinchVsPower(ros::NodeHandle *nh, bool using_clutter): 
        GraspListener(nh),
        file(), clutter(using_clutter), 
        id(0), file_path("/home/oscar/Desktop/pinch_vs_pow.txt"){

    grasp_client = nh->serviceClient<std_srvs::SetBool>("gr_exec");
    srv.request.data = true;

    pub = nh->advertise<std_msgs::String>("stop_flag", 10);
    msg.data = "";

    open_file();

    ros::param::get("target_object", object);
    std::cout << "The object being used is: " << object << std::endl;
    std::cout << "The clutter flag has been set to: " << clutter << std::endl;
}

PinchVsPower::~PinchVsPower(){
    file->close();
    std::cout << "The file has been saved at: " << file_path << std::endl;
}

void PinchVsPower::open_file(){
    file= new std::ofstream(file_path); // opens the file

    if(!file->is_open()) { // file couldn't be opened
      std::cerr << "Error1: file could not be opened" << std::endl;
      exit(1);
    }

    *file << "id, Grasp, Width, Object, Success, Clutter" << std::endl;
}

void PinchVsPower::register_data(){
    set_data();

    *file 
    << id << ", " 
    << grasp << ", " 
    << success << ", " 
    << width << ", " 
    << object << ", " 
    << clutter << std::endl;
}

void PinchVsPower::set_data(){
    id++;
    width = get_width();
    set_grasp();

    send_grasp_request();
    set_success();
    pub.publish(msg);
    std::cout << "Press enter when object is detected again:";
    std::string enter;
    std::cin >> enter;
}

void PinchVsPower::set_grasp(){
    bool pw_g_f =  get_power_gr_flag();
    if(pw_g_f == true){
        grasp == "power";
    }
    else{
        grasp == "pinch";
    }
}

void PinchVsPower::send_grasp_request(){
    if (grasp_client.call(srv))
    {
        ROS_INFO_STREAM("The grasp execution request was properly sent.");
    }
    else
    {
        ROS_ERROR("Failed to call service gr_exec");
    }
}

void PinchVsPower::set_success(){
    std::string succeded = "";
    do{
        std::cout << "Enter y if grasp succeded or n if not:";
        std::cin >> succeded;
    }
    while(succeded != "n" && succeded != "y");   
    if(succeded == "n"){
        success = false;
    }
    else if(succeded == "y"){
        success = true;
    }
}