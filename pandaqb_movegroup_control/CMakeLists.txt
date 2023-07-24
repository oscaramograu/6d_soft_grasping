cmake_minimum_required(VERSION 3.0.2)
project(pandaqb_movegroup_control)

find_package(catkin REQUIRED COMPONENTS
  tf
  tf_conversions
  roscpp
  std_msgs
  # message_generation
  moveit_core 
  moveit_ros_planning 
  moveit_ros_planning_interface
  # moveit_ros_perception
  moveit_visual_tools
  # cv_bridge
  # urdf  
)

find_package(PCL REQUIRED)

# generate_messages(
#   DEPENDENCIES
#   std_msgs
#   sensor_msgs
#   geometry_msgs
# )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES 
    Control_libs
    MoveGroup_libs
  CATKIN_DEPENDS 
    tf
    tf_conversions
    # rospy
    roscpp 
    std_msgs
    moveit_core 
    moveit_ros_planning 
    moveit_ros_planning_interface
    # moveit_ros_perception
    moveit_visual_tools
    # cv_bridge
 
  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  # ${Boost_INCLUDE_DIR}
  # ${moveit_ros_planning_interface_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
# ======================================== Controller Node ========================================
add_library(MoveGroup_libs
  ${PROJECT_SOURCE_DIR}/src/MoveGroup/BaseMoveGroup.cpp
  ${PROJECT_SOURCE_DIR}/src/MoveGroup/GroupMover.cpp
  ${PROJECT_SOURCE_DIR}/src/MoveGroup/InfoTriggerMG.cpp
)
add_library(Control_libs
  ${PROJECT_SOURCE_DIR}/src/Control/HandController.cpp
  ${PROJECT_SOURCE_DIR}/src/Control/ArmController.cpp
)

add_executable(controller_node 
  ${PROJECT_SOURCE_DIR}/src/controller_node.cpp
)

add_dependencies(controller_node ${controller_node_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(Control_libs ${catkin_EXPORTED_TARGETS})
add_dependencies(MoveGroup_libs ${catkin_EXPORTED_TARGETS})

target_link_libraries(MoveGroup_libs
  ${catkin_LIBRARIES}
  # ${Boost_LIBRARIES}
  # ${moveit_ros_planning_interface_LIBRARIES}
  ${PCL_LIBRARIES}
)
target_link_libraries(controller_node
  Control_libs
  MoveGroup_libs
)
# ======================================== Object Publisher Node ========================================
add_library(Object_libs
  ${PROJECT_SOURCE_DIR}/src/Target/TargetObject.cpp
  ${PROJECT_SOURCE_DIR}/src/Target/TargetMeshBroadcaster.cpp
)

add_executable(object_publisher_node 
  ${PROJECT_SOURCE_DIR}/src/object_publisher_node.cpp
)

add_dependencies(object_publisher_node ${object_publisher_node_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(Object_libs ${catkin_EXPORTED_TARGETS})

target_link_libraries(Object_libs
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
target_link_libraries(object_publisher_node
  Object_libs
)

add_executable(scene_objects_publisher_node 
  ${PROJECT_SOURCE_DIR}/src/scene_objects_publisher.cpp
)
add_dependencies(scene_objects_publisher_node ${scene_objects_publisher_node_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(scene_objects_publisher_node Object_libs)
