#include <ros/package.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <pcl/io/ply_io.h>

class ObjectBase
{
private:
    std::string object_name_, mesh_path;
    shape_msgs::Mesh moveit_mesh;
    moveit_msgs::CollisionObject collision_object;

    moveit_msgs::PlanningScene planning_scene;
    ros::NodeHandle nh;
    ros::Publisher planning_scene_diff_publisher;

    void set_mesh_path();
    void load_moveit_mesh();
    void create_collision_object(geometry_msgs::Pose pose);

public:
    ObjectBase(const std::string& object_name);
    ~ObjectBase();

    void add_to_world(geometry_msgs::Pose pose);
};