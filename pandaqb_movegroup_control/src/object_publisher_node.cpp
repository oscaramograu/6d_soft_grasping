#include <ros/ros.h>
#include <pandaqb_movegroup_control/Target/TargetMeshBroadcaster.h>

std::string grippr_mesh_pth(){
    std::string impose_grasp_path, mesh_folder, file_name;

    impose_grasp_path = ros::package::getPath("impose_grasp");
    mesh_folder =  impose_grasp_path + "/data/models/";
    file_name = "hand_col.ply";

    return mesh_folder + file_name;
}
std::string hand_path = grippr_mesh_pth();

int main(int argc, char** argv){
    ros::init(argc, argv, "object_publisher_node");

// ==============  PUBLISH THE TARGET OBJECT ============== 
    TargetMeshBr object("", "cpsduck");
    object.add_target_mesh(true);

// ============== PUBLISH THE HAND COLLISION MODEL ============== 
    // TargetMeshBr object(hand_path, "hand_model");
    // object.add_target_mesh(false);

    ros::spin();
    ros::shutdown();
    return 0;
}