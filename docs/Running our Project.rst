=====
How to run our project in Gazebo
=====

In your virtual machine (see previous sections for instructions on download and setup), download the track_ik inverse kinematic solver (https://bitbucket.org/traclabs/trac_ik/src/HEAD/trac_ik_python/, more details in the IK Solver documentation).

Finding our scripts
=====

Go to https://github.com/gabrieledamone/staircase/tree/master/_Planning. Copy and paste the files stairecaseReal.py and map.py into your virtual machine in the following directory:

catkin_ws/src/franka_gazebo/scripts

Running the project
=====

In a terminal window, launch an empty Gazebo world.

roslaunch gazebo_ros empty_world.launch

In another terminal, launch the Franka arm.

roslaunch franka_gazebo panda_arm_hand.launch

Set up four piles of bricks according to the positions found in the map.py file in *brickPiles*.



In a new terminal, type:

catkin_ws/src/franka_gazebo/scripts/stairecaseReal.py

Input the number of steps you want your staircase to have - maximum 5.

