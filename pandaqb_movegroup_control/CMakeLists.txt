cmake_minimum_required(VERSION 3.0.2)
project(pandaqb_movegroup_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
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
  GraspingService.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES Panda_Libraries
  CATKIN_DEPENDS 
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

add_library(Panda_Libraries
  ${PROJECT_SOURCE_DIR}/src/ControllerAPIs/move_group.cpp
  ${PROJECT_SOURCE_DIR}/src/ControllerAPIs/target_approacher.cpp
  ${PROJECT_SOURCE_DIR}/src/CameraAPIs/image_handler.cpp
)

add_executable(test_target_approacher 
  ${PROJECT_SOURCE_DIR}/src/test_target_approacher.cpp

  ${PROJECT_SOURCE_DIR}/src/ControllerAPIs/move_group.cpp
  ${PROJECT_SOURCE_DIR}/src/ControllerAPIs/target_approacher.cpp
  ${PROJECT_SOURCE_DIR}/src/CameraAPIs/image_handler.cpp
)

add_executable(grasp_server 
  src/grasp_server.cpp
)

add_dependencies(test_target_approacher ${test_target_approacher_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(Panda_Libraries ${catkin_EXPORTED_TARGETS})

target_link_libraries(Panda_Libraries
  Eigen3::Eigen
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${moveit_ros_planning_interface_LIBRARIES}
)

target_link_libraries(test_target_approacher
  Eigen3::Eigen
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${moveit_ros_planning_interface_LIBRARIES}
)

target_link_libraries(grasp_server
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)