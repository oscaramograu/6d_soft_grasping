# ToDo's
- Hand Eye Calibration (get the camera cable, mount on the robot, print chess board on a solid base, use move it camera calibration and safe the poses).
- Change the used model of the cat for the duck model.
- Make a better way to select the model in impose grasp.
- Test image detection of the ducks using 6IMPOSE.
- Test if the 6D positioning works properly.
- Write the pipeline to execute a grasp.
- Requirements.txt for impose grasp.

# Install all the dependencies in Linux:
## General requirements
- Install CUDA 11.2 at least.

- Install ROS Noetic: http://wiki.ros.org/ROS/Installation

- Install MoveIt: https://moveit.ros.org/install/

- Install libfranka and franka_ros: https://frankaemika.github.io/docs/installation_linux.html

- Install the qb soft hand required packages: http://wiki.ros.org/qb_hand/Tutorials/ROS%20Packages%20Installation

- Clone the repository in your ros workspace inside the src folder: (if you don't have any workspace create one)
```
cd ~/catkin_ws/src
git clone https://github.com/oscaramograu/6d_soft_grasping.git
```

## Impose grasp package setup
- Create a anaconda environtment and activate it.

- Install the requirements.txt.
```
pip install -r requirements.txt
```

- Clone 6IMPOSE anywhere in your computer, and follow the instructions to run the demo file. https://github.com/HP-CAO/6IMPOSE

- Set the path to the cloned 6IMPOSE repo in impose_grasp package.


## Build project
- Source the workspace:
```
cd ~/catkin_ws/src
source devel/setup.bash
```

- Build the probject:
```
catkin_make
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

3 - Launch the Controller Node to execute the routine to perform the best grasp on the detected object: (**Still need to be developed**)
```
roslaunch pandaqb_movegroup_control controller_node.launch
```
