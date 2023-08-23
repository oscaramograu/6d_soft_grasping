#include <ros/ros.h>
#include <pandaqb_movegroup_control/Target/TargetMeshBroadcaster.h>

std::string grippr_mesh_pth(std::string robot_config){
    std::string impose_grasp_path, mesh_folder, extension;

    impose_grasp_path = ros::package::getPath("impose_grasp");
    mesh_folder =  impose_grasp_path + "/data/models/";

    if(robot_config == "qb_hand"){
        extension = "_col2.ply";
    }
    else{
        extension = "_col.ply";
    }
    return mesh_folder + robot_config + extension;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "object_publisher_node");
    std::string obj_name;
    ros::param::get("target_object", obj_name);

    std::string rbt_conf, hand_path;
    ros::param::get("robot_config", rbt_conf);
    hand_path = grippr_mesh_pth(rbt_conf);
// ==============  PUBLISH THE TARGET OBJECT ============== 
    TargetMeshBr object("", obj_name);
    object.add_target_mesh(true);

// ============== PUBLISH THE HAND COLLISION MODEL ============== 
    // TargetMeshBr object(hand_path, "eef");
    // object.add_target_mesh(false);

    ros::spin();
    ros::shutdown();
    return 0;
}