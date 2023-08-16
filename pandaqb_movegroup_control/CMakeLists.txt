cmake_minimum_required(VERSION 3.0.2)
project(pandaqb_movegroup_control)

find_package(catkin REQUIRED COMPONENTS
  tf
  tf_conversions
  roscpp
  # system_lib
  std_msgs
  # message_generation
  moveit_core 
  moveit_ros_planning 
  moveit_ros_planning_interface
  moveit_ros_perception
  moveit_visual_tools
  # cv_bridge
  # urdf  
)

find_package(PCL REQUIRED)
find_package(nlohmann_json 3.2.0 REQUIRED)

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
    Experiment_libs
  CATKIN_DEPENDS 
    tf
    tf_conversions
    # rospy
    roscpp 
    std_msgs
    moveit_core 
    moveit_ros_planning 
    moveit_ros_planning_interface
    moveit_ros_perception
    moveit_visual_tools
    # cv_bridge
 
  DEPENDS system_lib
)

include_directories(
  include
  include/third_party
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
  ${moveit_ros_planning_interface_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
add_library(MoveGroup_libs
  ${PROJECT_SOURCE_DIR}/src/MoveGroup/BaseMoveGroup.cpp
  ${PROJECT_SOURCE_DIR}/src/MoveGroup/GroupMover.cpp
  ${PROJECT_SOURCE_DIR}/src/MoveGroup/InfoTriggerMG.cpp
)
add_library(Control_libs
  ${PROJECT_SOURCE_DIR}/src/Control/EEFController.cpp
  ${PROJECT_SOURCE_DIR}/src/Control/HandController.cpp
  ${PROJECT_SOURCE_DIR}/src/Control/GripperController.cpp
  ${PROJECT_SOURCE_DIR}/src/Control/ArmController.cpp
  ${PROJECT_SOURCE_DIR}/src/Target/GraspListener.cpp
  ${PROJECT_SOURCE_DIR}/src/Control/Controller.cpp
)

add_library(Experiment_libs
  ${PROJECT_SOURCE_DIR}/src/Experiments/GraspRegisterer.cpp
  ${PROJECT_SOURCE_DIR}/src/Target/GraspListener.cpp
)

target_link_libraries(MoveGroup_libs
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${moveit_ros_planning_interface_LIBRARIES}
  ${PCL_LIBRARIES} 
  nlohmann_json::nlohmann_json
)

target_link_libraries(Experiment_libs
  ${catkin_LIBRARIES}
)

add_dependencies(Control_libs ${catkin_EXPORTED_TARGETS})
add_dependencies(MoveGroup_libs ${catkin_EXPORTED_TARGETS})
add_dependencies(Experiment_libs ${catkin_EXPORTED_TARGETS})

# ======================================== Controller Node ========================================
add_executable(controller_node 
  ${PROJECT_SOURCE_DIR}/src/controller_node.cpp
)

add_dependencies(controller_node ${controller_node_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(controller_node
  Control_libs
  MoveGroup_libs
)
# ======================================== Grasps Registerer Node ========================================
add_executable(grasps_registerer_node 
  ${PROJECT_SOURCE_DIR}/src/grasps_register_node.cpp)

add_dependencies(grasps_registerer_node 
  ${grasps_registerer_node_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(grasps_registerer_node
  MoveGroup_libs)

# ======================================== Camera Calibration Control Node ========================================
add_executable(camera_callibration_control_node 
  ${PROJECT_SOURCE_DIR}/src/camera_callibration_control.cpp)

add_dependencies(camera_callibration_control_node 
  ${camera_callibration_control_node_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(camera_callibration_control_node
  MoveGroup_libs)

# ======================================== Experiment Recorder Node ========================================
add_executable(experiment_recorder_node
  ${PROJECT_SOURCE_DIR}/src/experiment_recorder.cpp)

add_dependencies(experiment_recorder_node
  ${experiment_recorder_node_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

  target_link_libraries(experiment_recorder_node
  Experiment_libs
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
