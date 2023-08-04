#include <pandaqb_movegroup_control/MoveGroup/BaseMoveGroup.h>

BaseMoveGroup::BaseMoveGroup(std::string planning_group){
    PLANNING_GROUP = planning_group;
    try {
        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(PLANNING_GROUP);
        
        load_params();
        print_params();
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
    ROS_INFO_STREAM("The planning time has been set to: " << planning_time);
    move_group_->setPlanningTime(planning_time);
    move_group_->setNumPlanningAttempts(planning_attempts);
    move_group_->setMaxVelocityScalingFactor(velocity_scaling);
    move_group_->setMaxAccelerationScalingFactor(acceleration_scaling);
}

void BaseMoveGroup::print_params(){
    ROS_INFO_STREAM("Planning time: " << planning_time);
    ROS_INFO_STREAM("Planning attempts:" << planning_attempts);
    ROS_INFO_STREAM("Velocity scaling: " << velocity_scaling);
    ROS_INFO_STREAM("Acceleration scaling: " << acceleration_scaling);
}