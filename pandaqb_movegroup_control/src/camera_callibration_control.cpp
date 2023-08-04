#include <ros/ros.h>
#include <pandaqb_movegroup_control/MoveGroup/GroupMover.h>

// THE TARGET WAS PLACED HORIZONTALLY SO THAT THE FIRST 
// METAL ELEMENT IN THE RAILS WAS COVERED

std::vector<std::vector<double>> joint_configurations;

void load_params(ros::NodeHandle *nh){
    if (nh->hasParam("joint_values"))
        {
        XmlRpc::XmlRpcValue xml_joint_values;
        nh->getParam("joint_values", xml_joint_values);

        if (xml_joint_values.getType() == XmlRpc::XmlRpcValue::TypeArray)
        {
            joint_configurations.resize(xml_joint_values.size());

            for (int i = 0; i < xml_joint_values.size(); ++i)
            {
                if (xml_joint_values[i].getType() == XmlRpc::XmlRpcValue::TypeArray)
                {
                    joint_configurations[i].resize(xml_joint_values[i].size());

                    for (int j = 0; j < xml_joint_values[i].size(); ++j)
                    {
                        joint_configurations[i][j] = static_cast<double>(xml_joint_values[i][j]);
                    }
                }
            }
        }
    }

    std::cout << joint_configurations[0][0] << std::endl; 
}

int main(int argc, char** argv){
    ros::init(argc, argv, "controller_node");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::NodeHandle nh;
    load_params(&nh);

    GroupMover mover("arm");

    for(int i = 0; i < joint_configurations.size(); i++){
        std::vector <double> configuration = joint_configurations[i];

        ROS_INFO_STREAM("Pose number " << i << 
            " of " << joint_configurations.size() << ":");

        for(int j = 0; j < configuration.size(); j++){
            ROS_INFO_STREAM("joint_" << j+1 << ": " << configuration[j]);
        }

        mover.moveTo(configuration);
    }
}
