#include <pandaqb_movegroup_control/Experiments/GraspRegisterer.h>

int n = 0;
int max_n = 2;

int main(int argc, char** argv){
    ros::init(argc, argv, "experiment_recorder_node");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::NodeHandle nh;
    GraspRegisterer grasp_registerer(&nh);

    while(n < max_n){
        n++;
        grasp_registerer.register_data();
    }
}