#include <pandaqb_movegroup_control/Experiments/GraspRegisterer.h>

GraspRegisterer::GraspRegisterer(
    ros::NodeHandle *nh): 
        GraspListener(nh), id(0), file(),
        exp_files_path("/home/oscar/catkin_ws/src/thesis/results/"){

    grasp_client = nh->serviceClient<std_srvs::SetBool>("gr_exec");
    srv.request.data = true;

    pub = nh->advertise<std_msgs::String>("stop_flag", 10);
    msg.data = "";

    set_file_path();
    open_file();

    std::cout << "The object being used is: " << object << std::endl;
}

GraspRegisterer::~GraspRegisterer(){
    file->close();
    std::cout << "The file has been saved at: " << file_path << std::endl;
}

void GraspRegisterer::set_file_path(){
    ros::param::get("target_object", object);
    ros::param::get("robot_config", eef);

    file_path = exp_files_path + "/" + eef + "/" + object + ".txt";
}   

void GraspRegisterer::open_file(){
    file= new std::ofstream(file_path); // opens the file

    if(!file->is_open()) { // file couldn't be opened
      std::cerr << "Error1: file " << file_path <<
        " could not be opened" << std::endl;

      exit(1);
    }

    *file << "id, Width, Object, Success, Grasp type" << std::endl;
}

void GraspRegisterer::register_data(){
    set_data();

    *file 
    << id << ", " 
    << width << ", " 
    << object << ", " 
    << success << ", " 
    << grasp_type << ", " 
    << std::endl;
}

void GraspRegisterer::set_data(){
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

void GraspRegisterer::set_grasp(){
    if(eef == "qb_hand"){
    bool pw_g_f =  get_power_gr_flag();
        if(pw_g_f == true){
            grasp_type = "power";
        }
        else{
            grasp_type = "pinch";
        }
    }
    else{
        grasp_type = "parallel_plates";
    }
}

void GraspRegisterer::send_grasp_request(){
    if (grasp_client.call(srv))
    {
        ROS_INFO_STREAM("The grasp execution request was properly sent.");
    }
    else
    {
        ROS_ERROR("Failed to call service gr_exec");
    }
}

void GraspRegisterer::set_success(){
    std::string succeded = "";
    do{
        std::cout << "Enter y if grasp succeded or n if not:";
        std::cin >> succeded;
    }
    while(succeded != "n" && succeded != "y");   
    std::cout << "Succeeded value = " << succeded << std::endl;
    if(succeded == "n"){
        success = false;
    }
    else if(succeded == "y"){
        success = true;
    }
    std::cout << "Success value = " << success << std::endl;
}