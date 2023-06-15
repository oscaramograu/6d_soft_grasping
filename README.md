# ToDo's
- Hand Eye Calibration (get the camera cable, mount on the robot, print chess board on a solid base, use move it camera calibration and safe the poses).
- Change the used model of the cat for the duck model.
- Test image detection of the ducks using 6IMPOSE.
- Test if the 6D positioning works properly.
- Write the pipeline to execute a grasp.

# Install all the dependencies in Linux:
- Install CUDA (at least 11.2)

- Create a conda environtment and install the requirements.txt

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

# Usage Instructions:
## Required Hardware
- Franka Emika Panda arm
- QB soft hand 2
- Realsense D415 camera
- Support for the camera _link required_ 
- Cable to connect the camera
- Duck toy _link required_

## Software
1 - Launch the franka with the qb hand in moveit to test the movements or using the real robot:
In one terminal launch the following:
- In simulation:
```
roslaunch pandaqb_movegroup_control panda_qb.launch
```

- Real robot:
```
roslaunch pandaqb_movegroup_control panda_qb.launch sim:=false
```

2 - Launch the 6IMPOSE detection node:
In a second terminal launch the following (after activating the conda environtment and installed the requirements.txt):

```
roslaunch impose_grasp Rt_publisher.launch
```

3 - Launch the Controller Node to execute the routine to perform the best grasp on the detected object:
```
roslaunch pandaqb_movegroup_control controller_node.launch
```
