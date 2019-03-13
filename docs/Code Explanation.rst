====
Code Explanation
====

Objectives
====

As discussed previously we have designed the functionnalites of the final code to be program by **individual groups** and then be put together at a later date.
The below features are *key* in our programming execution and what we understand as *milestones* in our work.

#. Pick & Placing

.. literalinclude:: ../_Planning/stairecaseReal.py
  :lines: 385-406
  
#. Pushing

#. Autonomous coding


Key Functions
====

Joint Move
----
The core joint planning function used was the "joint_move" function. Inputting the joint publishers of the robot arm and the desired end effector position moves the robot arm from its current end effector position to the desired position using the specified trajectory planning. A key improvement would be to give the user a choice of trajectory planning alogrithms (linear, trigonometric, polynomial, trapezoid velocity profile etc.) for different situations.

Currently, the joint move function employs the inverse kinematics function and moves the robot arm a single step in each iteration of the task space trajectory. This meant that the robot arm was being moved without reference to the entire joint space trajectory. By performing the inverse kinematics on the entire list of task space steps and then performing trajectory smoothing (low pass filter, damping) before the joint angles are published, the shaking could be eliminated from the robot arm movement.

Moving a Brick
----

Pickplace allows for a start position and end position to be input into the function and for all the necessary arm and gripper movements to be executed to move the brick from start to finish with uniaxial motion. The reason for uniaxial motion is so that there is much less chance of the interfering with obstacles as the end effector is lifted above the construction before any horizontal movement. Note that the horizontal movement can be duoaxial.
    
    
Moving the Gripper
----
As opposed to using the gripper width topic in order to control the gripper fingers, the fingers were controlled indiviudally, incase that ever became necessary during the task.
    
Debug Functions
----
A number of debugging functions were used in order to test various aspects of the code. 

The "franka_test" function comes from the example_joint_publisher.py script provided to us. It tests that the franka panda robot and simulation are working as well as ros (the topics and publishers for the robot arm and gripper). This was the the most basic test for functionality and further development.

The "sequence" function tests efficacy of the inverse kinematics solver. The function instructs the robot arm to move to a number (4) of positions to demonstrate that the inverse kinematic solver works irrespective of any trajectory planning. If the robot does not move through the positions but the "franka_test" fuction does work then the issue can be narrrowed to the inverse kinematics solver. The "sequence" function was hugely helpful in establishing that the inverse kinematics solver was sometimes returning unsatisfactory outputs (made the movement unstable and took long routes on occasion) and led to us realising that we were using an abritrary seed state (inital state) for the inverse kinematics as opposed to the current position.

The "joint_move_test" function test the trajcotry planning function. It is essentially the same as the "sequence" function except with a slower, smoother movement. A comparison of this and the "sequence" function is very helpful in demonstrating the efficacy of the trajectory planning.

The "pick_brick" function tests the robot arm picking up a single brick. As well as testing the for all the aforementioned functions, this funciton crucially also tests the Gazebo simulation physics and interactions between the brick and the grippers. The "pick_brick" function also was used to find the correct Cartesian end-effector orientation to be converted to a quaternion to input into the IK solver. This function was also used in order to determine issues
