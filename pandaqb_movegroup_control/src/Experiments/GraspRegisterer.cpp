#include <pandaqb_movegroup_control/Experiments/GraspRegisterer.h>

GraspRegisterer::GraspRegisterer(ros::NodeHandle *nh): 
        Controller(nh), id(0), file(),
        exp_files_path("/home/oscar/catkin_ws/src/thesis/results/"){

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

    *file << "id, Width, Object, Grasp Success, Place Success, First sinergy, Second sinergy" << std::endl;
}

void GraspRegisterer::register_data(){
    set_data();

    *file 
    << id << ", " 
    << width << ", " 
    << object << ", " 
    << grasp_success << ", " 
    << place_success << ", " 
    << sinergies[0] << ", " 
    << sinergies[1] << ", "
    << std::endl;
}

void GraspRegisterer::set_data(){
    id++;
    width = get_width();
    sinergies = get_sinergies();
    
    start_new_routine();

    std::cout << "GRASP SUCCESS";
    set_success(grasp_success);
    std::cout << "PLACE SUCCESS";
    set_success(place_success);

    pub.publish(msg);
    std::cout << "Press enter when object is detected again:";
    std::string enter;
    std::getline(std::cin, enter);
}

void GraspRegisterer::set_success(bool &successs_var){
    std::string succeded = "";
    do{
        std::cout << "Enter y if succeded or n if not:";
        std::cin >> succeded;
    }
    while(succeded != "n" && succeded != "y");   
    std::cout << "Succeeded value = " << succeded << std::endl;
    if(succeded == "n"){
        successs_var = false;
    }
    else if(succeded == "y"){
        successs_var = true;
    }
    std::cout << "Success value = " << successs_var << std::endl;
}