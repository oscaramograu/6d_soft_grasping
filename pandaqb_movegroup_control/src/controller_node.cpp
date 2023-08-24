#include <ros/ros.h>
// #include <pandaqb_movegroup_control/Target/TargetMeshBroadcaster.h>
// #include <pandaqb_movegroup_control/MoveGroup/GroupMover.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
// #include <pandaqb_movegroup_control/Control/EEFController.h>
// #include <pandaqb_movegroup_control/Control/ArmController.h>
// #include <pandaqb_movegroup_control/Target/GraspListener.h>
#include <pandaqb_movegroup_control/Control/Controller.h>

void rotate_aroundZ(tf::StampedTransform &tf, float theta){
    tf::Quaternion tf_rot;
    tf_rot.setW(cos(theta/2));
    tf_rot.setX(0);
    tf_rot.setY(0);
    tf_rot.setZ(sin(theta/2));
    
    tf::StampedTransform tf_Z;
    tf_Z.setRotation(tf_rot);
    tf*=tf_Z;
}

void rotate_aroundX(tf::StampedTransform &tf, float theta){
    tf::Quaternion tf_rot;
    tf_rot.setW(cos(theta/2));
    tf_rot.setX(sin(theta/2));
    tf_rot.setY(0);
    tf_rot.setZ(0);
    
    tf::StampedTransform tf_X;
    tf_X.setRotation(tf_rot);
    tf*=tf_X;
}

void offset_Z(tf::StampedTransform &tf, float offset){
    tf::Vector3 tf_pose(0, 0, offset);
    
    tf::StampedTransform tf_Z;
    tf_Z.setOrigin(tf_pose);
    tf*=tf_Z;
}


geometry_msgs::Pose tf_to_pose(tf::StampedTransform transform){
    geometry_msgs::Pose pose;
    tf::Vector3 position = transform.getOrigin();
    pose.position.x = position.getX();
    pose.position.y = position.getY();
    pose.position.z = position.getZ();

    tf::Quaternion rotation = transform.getRotation();
    pose.orientation.w = rotation.getW();
    pose.orientation.x = rotation.getX();
    pose.orientation.y = rotation.getY();
    pose.orientation.z = rotation.getZ();

    return pose;
}

int main(int argc, char** argv){
    // Initialize the node
    ros::init(argc, argv, "controller_node");

    ros::AsyncSpinner spinner(1);
    spinner.start();

// ================= REAL CODE ===========================
    // ros::NodeHandle nh;
    // GraspListener gr(&nh);
    // geometry_msgs::Pose target_pose = gr.get_grasp_pose();

    // ArmController ac;
    // EEFController eef_c(&nh);
    
    // ac.set_grasp(target_pose);

    // eef_c.open();

    // ac.approach_grasp();

    // std::string s = ";";
    // while (s == ";"){
    //     ac.move_to_g_pose();
    //     std::cout << "Type ; to keep iterating: ";
    //     std::cin >> s;
    //     target_pose = gr.get_grasp_pose();
    //     ac.set_grasp(target_pose);
    // }

    // eef_c.close();
    
    // ac.pick_up();

    // eef_c.open();

// ================= EXPERIMENTING ===========================
    // Controller controller(&nh);
    // ROS_INFO_STREAM("READY TO GRASP");

    // ros::Rate loop_rate(10);  // Adjust the rate as needed
    // while (ros::ok())
    // {
    //     ros::spinOnce();
    //     loop_rate.sleep();
    // }
// ================= TESTING POSE ===========================
    // tf::StampedTransform transform;
    // tf::TransformListener listener;

    // try {
    //     listener.waitForTransform("/panda_link0", "/cpsduck_frame", 
    //         ros::Time(0), ros::Duration(50.0));
    //     listener.lookupTransform("/panda_link0", "/cpsduck_frame", 
    //         ros::Time(0), transform);
    // } catch (tf::TransformException ex) {
    //     ROS_ERROR("%s",ex.what());
    // }

    // tf::Quaternion rotation;
    // rotation.setW(0);
    // rotation.setX(0);
    // rotation.setY(0);
    // rotation.setZ(1);

    // transform.setRotation(rotation);
    // rotate_aroundX(transform, M_PI);

    // geometry_msgs::Pose  target_pose = tf_to_pose(transform);
    // target_pose.position.z += 0.05;

    // GroupMover arm("arm");
    // arm.set_EEF_link("qbhand2m1_end_effector_link");
    // arm.moveTo(target_pose);
    
// ================= PRINT CURRENT JOINT STATES ===========================

    GroupMover arm("arm");
    arm.set_EEF_link("panda_link8");
    arm.printCurrentJointPosition();

    // target_pose.position.z += 0.05;

    // ROS_INFO_STREAM("The target pose is: " << target_pose);
    // arm.moveTo(target_pose);

// ================= TEST CLASSES ===========================
    // GripperController gc;
    // gc.close_gripper(0.00);
    // gc.close_gripper(0.02);

    // gc.open_gripper();
    // gc.printCurrentJointPosition();
}