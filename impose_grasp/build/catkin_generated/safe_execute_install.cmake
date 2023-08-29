execute_process(COMMAND "/home/neurolab/catkin_ws/src/thesis/impose_grasp/build/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/neurolab/catkin_ws/src/thesis/impose_grasp/build/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
