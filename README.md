# ToDos
- Download dataset from Danieles pc

- Train model in Daniele's pc

- Install CUDA toolkit in workstation pc

- Test zed library depth computation

- Copy model from Daniele's pc to workstation

- Send depth image to model and check the output

# Install all the dependencies in Linux:
- Install ROS Noetic: http://wiki.ros.org/ROS/Installation

- Install MoveIt: https://moveit.ros.org/install/

- Install libfranka and franka_ros: https://frankaemika.github.io/docs/installation_linux.html

- Open a terminal and create a ros workspace in your desired location and clone the repository inside src folder:
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/oscaramograu/thesis.git
cd ..
catkin_make
```

- Source the workspace:
```
source devel/setup.bash
```

- Remember that to avoid having to source the ```~/catkin_ws/devel/setup.bash``` each time you open a new shell, you can add the ```“source ~/catkin_ws/devel/setup.bash”``` on your ```~/.bashrc```.

# To use the code:
1 - Launch the robot:

In simulation:
```
roslaunch pandaqb_movegroup_control robot_only.launch
```

Real robot:
```
roslaunch pandaqb_movegroup_control robot_only.launch use_fake_hardware:=false
```

2 - Launch the NN Node and the Camera Node:
```
roslaunch pandaqb_movegroup_control dummy_nn_node.launch
```

3 - Launch the Controller Node:
```
roslaunch pandaqb_movegroup_control controller_node.launch
```
