#include <Eigen/Dense>
#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>

class ObjWorldTransformer{
public:
    ObjWorldTransformer();
    ~ObjWorldTransformer();

    void set_ObjCam_T(Eigen::Affine3d obj_cam);
    void set_CamEEF_T(Eigen::Affine3d CamEEF);
    void set_EEFWorld_T(Eigen::Affine3d EEF_w);

protected:
    void compute_Obj_to_World_transformation();
    Eigen::Affine3d* get_ObjWorld_T();

private:
    Eigen::Affine3d *obj_cam_, *cam_EEF_, *EEF_w_, *obj_w_;

    void set_ObjWorld_T(Eigen::Affine3d obj_w);
    Eigen::Affine3d compose(Eigen::Affine3d* AB, Eigen::Affine3d* BC);
};