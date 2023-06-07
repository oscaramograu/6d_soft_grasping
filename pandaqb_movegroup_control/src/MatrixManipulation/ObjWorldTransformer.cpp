#include <pandaqb_movegroup_control/MatrixManipulation/ObjWorldTransformer.h>

ObjWorldTransformer::ObjWorldTransformer(){
}
ObjWorldTransformer::~ObjWorldTransformer(){
}

void ObjWorldTransformer::set_ObjCam_T(Eigen::Affine3d obj_cam){
    *obj_cam_ = obj_cam;
}
void ObjWorldTransformer::set_CamEEF_T(Eigen::Affine3d cam_EEF){ 
    //LOAD THE MATRIX FROM THE PARAMETER SERVER
    *cam_EEF_ = cam_EEF;
}
void ObjWorldTransformer::set_EEFWorld_T(Eigen::Affine3d EEF_w){
    *EEF_w_ = EEF_w;
}
void ObjWorldTransformer::set_ObjWorld_T(Eigen::Affine3d obj_w){
    *obj_w_ = obj_w;
}

Eigen::Affine3d ObjWorldTransformer::compose(Eigen::Affine3d* AB, Eigen::Affine3d* BC){
    Eigen::Affine3d* AC;
    AC->linear() = AB->linear() * BC->linear();
    AC->translation() = AB->translation() + AB->linear() * BC->translation();

    return *AC;
}

void ObjWorldTransformer::compute_Obj_to_World_transformation(){
    Eigen::Affine3d obj_EEF, obj_w;
    obj_EEF = compose(obj_cam_, cam_EEF_);
    obj_w = compose(&obj_EEF, EEF_w_);

    set_ObjWorld_T(obj_w);
}

Eigen::Affine3d* ObjWorldTransformer::get_ObjWorld_T(){
    return obj_w_;
}