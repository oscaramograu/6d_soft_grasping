#include <pandaqb_movegroup_control/Target/TargetObject.h>

TargetObject::TargetObject(const std::string& object_name): object_name_(object_name){
    planning_scene_diff_publisher = nh.advertise<moveit_msgs::PlanningScene>(
        "planning_scene", 1);
    set_mesh_path();
    load_moveit_mesh();
}

TargetObject::~TargetObject(){
}

void TargetObject::add_to_world(geometry_msgs::Pose pose){
    create_collision_object(pose);
    
    planning_scene.world.collision_objects.push_back(collision_object);
    planning_scene.is_diff = true;
    planning_scene_diff_publisher.publish(planning_scene);
    ROS_INFO_STREAM(object_name_ << "Object added to world");
}

void TargetObject::set_mesh_path(){
    std::string impose_grasp_path, mesh_folder, file_name;

    impose_grasp_path = ros::package::getPath("impose_grasp");
    mesh_folder =  impose_grasp_path + "/data/models/" + object_name_ + "/";
    file_name = object_name_ + ".ply";

    mesh_path = mesh_folder + file_name;
}

void TargetObject::load_moveit_mesh(){
    // Load the mesh file using PCL
    pcl::PolygonMesh pcl_mesh;
    if (pcl::io::loadPLYFile(mesh_path, pcl_mesh) == -1) {
        ROS_ERROR("Failed to load PLY file");
    }

    // Convert vertices
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
    pcl::fromPCLPointCloud2(pcl_mesh.cloud, pcl_cloud);

    for (const auto& point : pcl_cloud.points) {
        geometry_msgs::Point vertex;
        vertex.x = point.x;
        vertex.y = point.y;
        vertex.z = point.z;
        moveit_mesh.vertices.push_back(vertex);
    }
    
    // Convert faces
    for (const auto& polygon : pcl_mesh.polygons) {
        if (polygon.vertices.size() != 3) {
            ROS_WARN("Only triangular faces are supported."
                "Skipping polygon with %zd vertices.", polygon.vertices.size());
            continue;
        }
        shape_msgs::MeshTriangle face;
        face.vertex_indices[0] = polygon.vertices[0];
        face.vertex_indices[1] = polygon.vertices[1];
        face.vertex_indices[2] = polygon.vertices[2];
        moveit_mesh.triangles.push_back(face);
    }

    ROS_INFO_STREAM("Mesh was loaded without errors");
}

void TargetObject::create_collision_object(geometry_msgs::Pose pose){
    // Create a collision object from the mesh
    collision_object.header.frame_id = "/panda_link0";
    collision_object.header.stamp = ros::Time::now();

    collision_object.id = object_name_;
    collision_object.meshes.push_back(moveit_mesh);
    collision_object.mesh_poses.push_back(pose);

    collision_object.operation = collision_object.ADD;   
    ROS_INFO_STREAM("Collision object created");
}