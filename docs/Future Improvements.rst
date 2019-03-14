====
Future Improvements
====
In the absence of technical problems due to *Gazebo*, the group would have accomplished the following.

Testing the code on the real robot
====

Our development process was significantly slowed by technical problems due to *Gazebo*, and we haven't had a chance to try our code on the physical robot.  We predict a much higher success rate in real life, as most of the problems that make our staircase ``break`` are due to the unrealistically low friction and weight of simulated bricks (see Gazebo Limitations page for details).

Improving the motion planning
====

Perfecting the interpolation of joint angles could reduce *shakiness* in the robot's movement (see Motion Planning page for details).

Error detection
====

The published position of the gripper's fingers can be used to detect if the end effector has failed to pick up or dropped a brick, and an error recovery protocol could be added.

First, asking the user whether they wish to continue building the staircase after the error occurred. If the robot has dropped a brick over the staircase it has built so far and moved it significantly as a result, the user may not with to continue.

If the error did not cause any movement of other bricks in the workspace, the user can input to continue building the staircase: the gripper is told to pick up the next brick in the column ``stations``, and to place it in the staircase position that the previous brick would have occupied had it not fallen.
