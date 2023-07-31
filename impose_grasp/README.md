Object detection:
    0- Wait for start signal
	1- Estimate obj pose with 6IMPOSE \
	2- Filter pose with last detected poses -
	3- Broadcast object pose tf \

Grasp selection:
    0- Wait for start signal
	1- Map registerd parallel plate grasps to soft hand. -
	2- Filter grasps that are reachable by the given pose of the object. -
	3- Select the best grasp out of reachable ones, given the obstruction around the object. -
	4- Broadcast the selected grasp and publish a boolean for pinch or power grasp. \

ToDos:
    1- Publish message with width openning
    2- Make a listener for the pose estimatioin to start detecting the object pose.
    3- Make a listener for the grasp selection to select a candidate and broadcast it.