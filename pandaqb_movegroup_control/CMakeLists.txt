cmake_minimum_required(VERSION 3.0.2)
project(pandaqb_movegroup_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_ros_perception
  moveit_visual_tools
  cv_bridge
)

find_package(Eigen3 REQUIRED)

add_service_files(
  FILES
  RequestGrasp.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES API_Libraries
  CATKIN_DEPENDS 
   rospy
   roscpp 
   std_msgs
   moveit_core 
   moveit_ros_planning 
   moveit_ros_planning_interface
   moveit_ros_perception
   moveit_visual_tools
   cv_bridge
 
  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
  ${moveit_ros_planning_interface_INCLUDE_DIRS}
)

add_library(API_Libraries
  ${PROJECT_SOURCE_DIR}/src/ControllerAPIs/move_group.cpp
  ${PROJECT_SOURCE_DIR}/src/ControllerAPIs/hand_group.cpp
  ${PROJECT_SOURCE_DIR}/src/ControllerAPIs/grasper.cpp
  ${PROJECT_SOURCE_DIR}/src/ControllerAPIs/controller.cpp

  ${PROJECT_SOURCE_DIR}/src/CameraAPIs/image_handler.cpp
  )

add_executable(controller_node 
  ${PROJECT_SOURCE_DIR}/src/controller_node.cpp

  ${PROJECT_SOURCE_DIR}/src/ControllerAPIs/move_group.cpp
  ${PROJECT_SOURCE_DIR}/src/ControllerAPIs/hand_group.cpp
  ${PROJECT_SOURCE_DIR}/src/ControllerAPIs/grasper.cpp
  ${PROJECT_SOURCE_DIR}/src/ControllerAPIs/controller.cpp

)

add_dependencies(controller_node ${controller_node_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_dependencies(API_Libraries ${catkin_EXPORTED_TARGETS})

target_link_libraries(API_Libraries
  Eigen3::Eigen
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${moveit_ros_planning_interface_LIBRARIES}
)

target_link_libraries(controller_node
  Eigen3::Eigen
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${moveit_ros_planning_interface_LIBRARIES}
)
