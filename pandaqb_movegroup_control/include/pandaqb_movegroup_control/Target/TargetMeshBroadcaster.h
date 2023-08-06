#include <pandaqb_movegroup_control/Target/TargetObject.h>
#include <tf/transform_listener.h>

class TargetMeshBr: public TargetObject{
private:
    tf::TransformListener listener;
    tf::Vector3 position;
    tf::Quaternion rotation;
    geometry_msgs::Pose pose;

    void listen();
    void listen2();

    void build_pose();

public:
    TargetMeshBr(const std::string& mesh_path, const std::string& object_name);
    ~TargetMeshBr();
    void add_target_mesh(bool is_obj);
};