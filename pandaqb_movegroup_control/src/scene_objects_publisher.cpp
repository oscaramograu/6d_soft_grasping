#include <ros/ros.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

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

    moveit_msgs::CollisionObject collision_object;

    collision_object.header.frame_id = "/panda_link0";
    collision_object.header.stamp = ros::Time::now();
    collision_object.id = "floor";

    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = 3;
    primitive.dimensions[primitive.BOX_Y] = 3;
    primitive.dimensions[primitive.BOX_Z] = 0.05;

    geometry_msgs::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = 0.0;
    box_pose.position.y = 0.0;
    box_pose.position.z = -0.03;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;
    
    moveit_msgs::PlanningScene planning_scene;
    planning_scene.world.collision_objects.push_back(collision_object);
    planning_scene.is_diff = true;
    planning_scene_diff_publisher.publish(planning_scene);


    ROS_INFO_STREAM("Object added to world");
    
    // Spin and process ROS callbacks
    ros::spin();
    ros::shutdown();
    return 0;
}