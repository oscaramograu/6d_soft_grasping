# franka-control Steps:

## 1-Lauch the system
### In the simulation:
Launch the file "simulated_panda_control.launch"
```
roslaunch franka_moveit_control simulated_panda_control.launch
```
### The real robot:
Launch the file "simulated_panda_control.launch"
```
roslaunch franka_moveit_control real_panda_control.launch
```

## 2- Run the control node
Modify the function "move_arm_to_specific_pose" to choose the desired pose.

Run "catkin_make" in the root of the workspace if you modified the desired pose.

In another terminal run the node "straight_move_node.cpp"
```
rosrun franka_moveit_control straight_move_node
```