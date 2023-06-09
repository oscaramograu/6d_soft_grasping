#include <pandaqb_movegroup_control/MatrixManipulation/obj_w_Rt_publisher.h>

ObjWorld_TPublisher::ObjWorld_TPublisher(){
    ObjCam_sub_ = nh_.subscribe("/obj_to_cam_Rt", 10, &ObjWorld_TPublisher::ObjCam_callback, this);
    
    // Get CamEEF_T from parameter server after HandEye calibration
    Eigen::Affine3d CamEEF_T;
    CamEEF_T.matrix().setIdentity();
    set_CamEEF_T(CamEEF_T);
}

ObjWorld_TPublisher::~ObjWorld_TPublisher(){
}

void ObjWorld_TPublisher::ObjCam_callback(const MultyArrayPtr &ObjCam_T_msg){
    Eigen::Affine3d ObjCam_T, EEFWorld_T;
    ObjCam_T = MultiArray_to_Affine(ObjCam_T_msg);
    EEFWorld_T = get_EEFWorld_T_from_tf();

    set_ObjCam_T(ObjCam_T);
    set_EEFWorld_T(EEFWorld_T);

    compute_Obj_to_World_transformation();

    Eigen::Affine3d ObjWorld_T = get_ObjWorld_T();

    tf::Transform tf;
    tf::transformEigenToTF(ObjWorld_T, tf);
    broadcast_tf(tf);

    ROS_INFO_STREAM("The ObjCam_T is: \n" << ObjCam_T.matrix());
    ROS_INFO_STREAM("The CamEEF_T is: \n" << get_CamEEF_T().matrix());
    ROS_INFO_STREAM("The EEFWorld_T is: \n" << EEFWorld_T.matrix());

    ROS_INFO_STREAM("The ObjWorld_T is: \n" << ObjWorld_T.matrix());
}

Eigen::Affine3d ObjWorld_TPublisher::MultiArray_to_Affine(const MultyArrayPtr &arr_msg) {
    std::vector<float> data_vec = arr_msg->data;
    Eigen::Matrix3d rotation_matrix;
    rotation_matrix << data_vec[0], data_vec[1], data_vec[2],
                    data_vec[3], data_vec[4], data_vec[5],
                    data_vec[6], data_vec[7], data_vec[8];

    Eigen::Vector3d translation_vector(data_vec[9], data_vec[10], data_vec[11]);

    Eigen::Affine3d affine_matrix = Eigen::Affine3d::Identity();

    affine_matrix.linear() = rotation_matrix;
    affine_matrix.translation() = translation_vector;
    return affine_matrix;
}

Eigen::Affine3d ObjWorld_TPublisher::get_EEFWorld_T_from_tf(){
    tf::TransformListener tfListener;
    tfListener.waitForTransform("world", "panda_hand_tcp", ros::Time(0), ros::Duration(5.0));

    tf::StampedTransform transform;      
    tfListener.lookupTransform("world", "panda_hand_tcp", ros::Time(0), transform);

    Eigen::Affine3d EEFWorld_T;
    tf::transformTFToEigen(transform, EEFWorld_T);

    return EEFWorld_T; 
}

void ObjWorld_TPublisher::broadcast_tf(tf::Transform ObjWorld_tf){
    static tf::TransformBroadcaster br;
    tf::StampedTransform tf_msg(
        ObjWorld_tf, ros::Time::now(), "world", "detected_obj"
        );
    br.sendTransform(tf_msg);
}