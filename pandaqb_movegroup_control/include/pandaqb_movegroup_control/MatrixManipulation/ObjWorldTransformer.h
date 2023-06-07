#include <Eigen/Dense>
#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>

class ObjWorldTransformer{
public:
    ObjWorldTransformer();
    ~ObjWorldTransformer();

    /**
     * sets the obj_cam transformation obtained from 
     * 6IMPOSE sent under the topic obj_cam_Rt. 
     * 
     * @param obj_cam object to camera transformation matrix   
    */
    void set_ObjCam_T(Eigen::Affine3d obj_cam);
    /**
     * sets the cam_EEF transformation obtained from 
     * the parameter server after performing 
     * hand-eye calibration. 
     * 
     * @param cam_EEF camera to end-effector 
     * transformation matrix  
    */
    void set_CamEEF_T(Eigen::Affine3d cam_EEF);
    /**
     * sets the EEF_w transformation obtained from 
     * the pandaqb movegroup. 
     * 
     * @param EEF_w end-effector to world 
     * transformation matrix  
    */
    void set_EEFWorld_T(Eigen::Affine3d EEF_w);

protected:
    /**
     * Once the ObjCam, CamEEF, and EEFWorld transformations
     * are set, it makes its commposition and calculates the
     * ObjWorld transformation. 
     * 
     * It finally sets this value to the obj_world_ matrix.
     */
    void compute_Obj_to_World_transformation();

    /**
     * Once the final object to world transformation matrix,
     * is computed, this function can return its value.
     * 
     * @return the final object to world transformation matrix
     */
    Eigen::Affine3d* get_ObjWorld_T();

private:
    Eigen::Affine3d *obj_cam_, *cam_EEF_, *EEF_w_, *obj_w_;

    void set_ObjWorld_T(Eigen::Affine3d obj_w);
    Eigen::Affine3d compose(Eigen::Affine3d* AB, Eigen::Affine3d* BC);
};