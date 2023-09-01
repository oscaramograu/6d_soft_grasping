#include <pandaqb_movegroup_control/Target/TargetObject.h>
#include <tf/transform_listener.h>
#include <ros/ros.h>

class TargetMeshBr: public TargetObject{
private:
    tf::TransformListener listener;
    tf::StampedTransform target_tf;
    geometry_msgs::Pose target_pose;

    tf::StampedTransform  listen(std::string child, std::string parent);
    tf::StampedTransform map_pose(tf::Vector3 target_pose, tf::Quaternion target_rot);
    geometry_msgs::Pose build_pose(tf::Vector3 position, tf::Quaternion rotation);

public:
    TargetMeshBr(const std::string& mesh_path, const std::string& object_name);
    ~TargetMeshBr();
    void add_target_mesh(bool is_obj);
};