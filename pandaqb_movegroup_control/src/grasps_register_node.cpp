#include <ros/ros.h>
#include <pandaqb_movegroup_control/MoveGroup/GroupMover.h>
#include <tf/transform_listener.h>
#include <third_party/nlohmann/json.hpp>

using json = nlohmann::json;

tf::StampedTransform gr_obj_Rt;
std::string nump;
float flag;
std::ofstream *outdata;
json data;

void open_file(std::string obj_name){
  std::string path = "/home/neurolab/catkin_ws/src/thesis/impose_grasp/data/models/"
    + obj_name + "/manual_registered_poses.json";
  outdata = new std::ofstream(path); // opens the file
  if(!outdata->is_open()) { // file couldn't be opened
    std::cerr << "Error1: file could not be opened" << std::endl;
    exit(1);
  }
}

void listen_gr_obj_tf(std::string target_name, tf::StampedTransform &transform){
  tf::TransformListener listener;
  try {
      listener.waitForTransform("/" + target_name + "_frame", "qbhand2m1_end_effector_link",
          ros::Time(0), ros::Duration(50.0));
      listener.lookupTransform( "/" + target_name + "_frame", "qbhand2m1_end_effector_link",
          ros::Time(0), transform);
  } catch (tf::TransformException ex) {
      ROS_ERROR("%s",ex.what());
  }
}

void build_np_str(){
  tfScalar pose_arr [16];

  gr_obj_Rt.getOpenGLMatrix(pose_arr);

  pose_arr[15] = 1;

  nump = "[";
  int r = 0, c = 0;
  for(int i = 0; i < 16; i ++){
    if (r==0){
      nump += "[";
    };
    if (r < 3){
      nump +=  std::to_string(pose_arr[c + r*4]) + ", ";
      r++;
    }
    else{
      nump += std::to_string(pose_arr[c + r*4]) + "]";
      c ++;
      r = 0;

      if(i!=15){
        nump += ",\n";
      }
    }
  }
  nump += "]";
}

void build_flag(){
  float i;

  do{
    std::cout << "Set the flag value 1 for power, 0 for pich:";
    std::cin >> i;
    if((i==1) | (i==0)){
      flag = i;
    }
    else{
      std::cout << "Wrong input, try again." << std::endl;
    }
  }
  while((i!=1) && (i!=0));
}

void add_grasp_to_file(){

  build_flag();
  build_np_str();

  if(outdata->is_open()) { 
    json pose = {
        { "pose",  nump},
        { "width", flag}
    };
    data.push_back(pose);
  }
}

int main(int argc, char **argv){
  ros::init(argc, argv, "move_group_interface_tutorial");
  std::string obj_name = "cpsduck";

  open_file(obj_name);
  data = json::array();

  int desired_gr = 2, n = 0;
  while(n<desired_gr){
    listen_gr_obj_tf(obj_name, gr_obj_Rt);

    add_grasp_to_file();
    n++;
  }

  *outdata << std::setw(4) << data << std::endl;
  outdata->close();

  ros::shutdown(); 
  return 0;
}
