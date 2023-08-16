#include <pandaqb_movegroup_control/Experiments/GraspRegisterer.h>

int n = 0;
int max_n = 2;

int main(int argc, char** argv){
    ros::init(argc, argv, "experiment_recorder_node");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::NodeHandle nh;
    bool using_clutter = true;
    bool using_mapped_grasps = true;
    ros::param::get("using_clutter", using_clutter);

    GraspRegisterer grasp_registerer(&nh, using_clutter, using_mapped_grasps);

    while(n < max_n){
        n++;
        grasp_registerer.register_data();
    }
}