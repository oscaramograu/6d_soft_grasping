# ToDos
- load data from the yamol file to the robot_only.launch file
- get the middpoint of the hand, and its normal
- reorganize the classes and interfaces:
    - It should be able to choose to load the normal gripper or the qb hand.
    - Divide the code in more clear classes and interfaces. More and more simple functionalities.
    - Depending on the EEF a different approach() must be designed. 
    - To approach the hand, it must move its normal must match the normal of the target grasp point.