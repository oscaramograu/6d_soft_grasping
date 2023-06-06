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
  urdf
)

# find_package(Eigen REQUIRED)
# include_directories(${Eigen_INCLUDE_DIRS})


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
  LIBRARIES 
    Controller_libs  
    MoveGroup_libs
    Matrix_libs
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

# ======================================== Controller Node ========================================
add_library(Controller_libs
  ${PROJECT_SOURCE_DIR}/src/ControllerAPIs/move_group.cpp
  ${PROJECT_SOURCE_DIR}/src/ControllerAPIs/hand_group.cpp
  ${PROJECT_SOURCE_DIR}/src/ControllerAPIs/manipulator.cpp
  ${PROJECT_SOURCE_DIR}/src/ControllerAPIs/controller.cpp
)
add_library(MoveGroup_libs
  ${PROJECT_SOURCE_DIR}/src/MoveGroup/BaseMoveGroup.cpp
  ${PROJECT_SOURCE_DIR}/src/MoveGroup/InfoTriggerMG.cpp
  ${PROJECT_SOURCE_DIR}/src/MoveGroup/GroupMover.cpp
  ${PROJECT_SOURCE_DIR}/src/MoveGroup/RobotMover.cpp
)

add_executable(controller_node 
  ${PROJECT_SOURCE_DIR}/src/controller_node.cpp
)

add_dependencies(controller_node ${controller_node_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(Controller_libs ${catkin_EXPORTED_TARGETS})
add_dependencies(MoveGroup_libs ${catkin_EXPORTED_TARGETS})

target_link_libraries(MoveGroup_libs
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${moveit_ros_planning_interface_LIBRARIES}
)
target_link_libraries(controller_node
  Controller_libs
  MoveGroup_libs
)

# ====================================== Matrix Publisher Node ======================================
add_library(Matrix_libs
  ${PROJECT_SOURCE_DIR}/src/MatrixManipulation/obj_w_Rt_publisher.cpp
)
add_executable(matrix_publisher_node 
  ${PROJECT_SOURCE_DIR}/src/matrix_publisher_node.cpp
)

add_dependencies(matrix_publisher_node ${matrix_publisher_node_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(Matrix_libs ${catkin_EXPORTED_TARGETS})

# target_link_libraries(Matrix_libs
#   Eigen3::Eigen
# )

target_link_libraries(matrix_publisher_node
  ${catkin_LIBRARIES}
  # ${Eigen_LIBRARIES}   
  Matrix_libs
)

# target_link_libraries(API_Libraries
#   Eigen3::Eigen
#   ${catkin_LIBRARIES}
#   ${Boost_LIBRARIES}
#   ${moveit_ros_planning_interface_LIBRARIES}
# )
