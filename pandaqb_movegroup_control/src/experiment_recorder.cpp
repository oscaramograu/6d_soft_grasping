#include <pandaqb_movegroup_control/Experiments/PinchVsPower.h>

int n = 0;
int max_n = 2;

int main(int argc, char** argv){
    ros::init(argc, argv, "experiment_recorder_node");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::NodeHandle nh;
    bool using_clutter;
    ros::param::get("using_clutter", using_clutter);

    PinchVsPower pinch_vs_pw(&nh, using_clutter);

    while(n < max_n){
        n++;
        pinch_vs_pw.register_data();
    }
}