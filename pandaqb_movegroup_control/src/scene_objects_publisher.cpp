#include <ros/ros.h>
#include <iostream>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

moveit_msgs::CollisionObject build_col_obj(std::string id, 
    shape_msgs::SolidPrimitive primitive, geometry_msgs::Pose pose){
    moveit_msgs::CollisionObject collision_object;

    collision_object.header.frame_id = "/panda_link0";
    collision_object.header.stamp = ros::Time::now();
    collision_object.id = id;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(pose);
    collision_object.operation = collision_object.ADD;

    return collision_object;
    }

shape_msgs::SolidPrimitive build_primitive(float x, float y, float z){
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = x;
    primitive.dimensions[primitive.BOX_Y] = y;
    primitive.dimensions[primitive.BOX_Z] = z;

    return primitive;
}

geometry_msgs::Pose build_pose(float x, float y, float z, shape_msgs::SolidPrimitive prim){
    geometry_msgs::Pose pose;
    pose.orientation.w = 1.0;
    pose.position.x = x + prim.BOX_X/2;
    pose.position.y = y + prim.BOX_Y/2;
    // pose.position.z = z + prim.BOX_Z/2;

    return pose;
}

moveit_msgs::CollisionObject build_box(){
    shape_msgs::SolidPrimitive box_prim = build_primitive(0.28, 0.28, 0.15);
    geometry_msgs::Pose box_pose = build_pose(0.55, 0.1, 0, box_prim);

    return build_col_obj("box", box_prim, box_pose);
}

moveit_msgs::CollisionObject build_platform(){
    shape_msgs::SolidPrimitive plat_prim = build_primitive(1.5, 1.5, 0.03);
    geometry_msgs::Pose plat_pose = build_pose(0.83, 0, 0.0, plat_prim);

    return build_col_obj("box", plat_prim, plat_pose);
}

int main(int argc, char **argv){
    // Initialize the node
    ros::init(argc, argv, "scene_objects_publisher_node");

    ros::NodeHandle n;

    ros::Publisher planning_scene_diff_publisher = n.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
    while(planning_scene_diff_publisher.getNumSubscribers() < 1)
    {
        ros::WallDuration sleep_t(0.5);
        sleep_t.sleep();
    }

    moveit_msgs::CollisionObject box, plat;
    box = build_box();
    plat = build_platform();

    moveit_msgs::PlanningScene planning_scene;
    planning_scene.world.collision_objects.push_back(box);
    planning_scene.world.collision_objects.push_back(plat);

    planning_scene.is_diff = true;
    planning_scene_diff_publisher.publish(planning_scene);


    ROS_INFO_STREAM("Object added to world");
    
    // Spin and process ROS callbacks
    ros::spin();
    ros::shutdown();
    return 0;
}