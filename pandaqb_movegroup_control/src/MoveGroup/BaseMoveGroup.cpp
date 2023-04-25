#include <pandaqb_movegroup_control/MoveGroup/BaseMoveGroup.h>

BaseMoveGroup::BaseMoveGroup(std::string planning_group){
    PLANNING_GROUP = planning_group;
    try {
        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(PLANNING_GROUP);

        load_params();
        }
    catch (const ros::InvalidNameException& e) {
    // Invalid param name
    ROS_ERROR_STREAM("Error: " << e.what());
    }
    catch (const ros::InvalidParameterException& e) {
    // Invalid param value
    ROS_ERROR_STREAM("Error: " << e.what());
    }
    catch (const ros::Exception& e) {
    // Another ros error
    ROS_ERROR_STREAM("Error: " << e.what());
    }
    catch (const std::exception& e) {
    // Another generic error
    ROS_ERROR_STREAM("Error: " << e.what());
    } catch (...) {
    // Any other error
    ROS_ERROR_STREAM("Error desconocido.");
    }
}

BaseMoveGroup::~BaseMoveGroup(){
    
}

void BaseMoveGroup::load_params(){
    std::string path = "move_group/" + PLANNING_GROUP + "/";

    ros::param::get(path + "home", home);
    ros::param::get(path + "planning_time", planning_time);
    ros::param::get(path + "planning_attempts", planning_attempts);
    ros::param::get(path + "velocity_scaling", velocity_scaling);
    ros::param::get(path + "acceleration_scaling", acceleration_scaling);

    move_group_->setPlanningTime(planning_time);
    move_group_->setNumPlanningAttempts(planning_attempts);
    move_group_->setMaxVelocityScalingFactor(velocity_scaling);
    move_group_->setMaxAccelerationScalingFactor(acceleration_scaling);
}

void BaseMoveGroup::add_collision_object(std::string name, 
        shape_msgs::SolidPrimitive primitive, geometry_msgs::Pose pose){
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = move_group_->getPlanningFrame();

    collision_object.id = name;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(pose);
    collision_object.operation = collision_object.ADD;

    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.push_back(collision_object);

    ROS_INFO_STREAM("Add an object into the world");
    planning_scene_.addCollisionObjects(collision_objects);
}