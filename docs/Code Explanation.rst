====
Code Explanation
====
Let's have a look at our final code: it's main objectives, structure and key functions within it.

Objectives
====
As discussed previously we have designed the functionalites of the final code to be programmed by **individual groups** and then be put together at a later date to produce the **result** we have today.
The below features are *key* in our programming execution and what we understand as *milestones* in our work.

Pick & Placing
----
A first step in our code is to make sure that we are able to pick and place bricks in order to build a staircase.
   
Pushing
----
A second step is to push bricks accordingly at the end when the whole staircase is built.

Autonomous staircase building
----
Finally we need to construct a code structure that makes the robot autonomous:

* it can build different heights of staircases
* has undergone a specific workplace planning and has knowledge on where bricks are
* identifies issues and can react to them to a certain extent

Overall Structure
====

1. It all starts with the user that can key in the ``height`` that he wants the staircase to have.

2. Then are loaded the relevant ``workspace maps`` for this specific ``height`` ie. the **initial** and **final** positions of the bricks.

3. Finally a ``while loop`` is run which looks like this::

      while loop

This while loop places all the bricks and runs the following 3 functions:

* pickPlace::

   function

* Push::

   function

* ErrorCatching::

   function
   
   
.. literalinclude:: ../_Planning/staircaseReal.py
   :lines:


Key Functions
====

Going further from our objectives we came to define key functions in our codes functioning that we are visiting below.

Joint Move
----
The core joint planning function used was the "joint_move" function. Inputting the joint publishers of the robot arm and the desired end effector position moves the robot arm from its current end effector position to the desired position using the specified trajectory planning. A key improvement would be to give the user a choice of trajectory planning alogrithms (linear, trigonometric, polynomial, trapezoid velocity profile etc.) for different situations.

Currently, the joint move function employs the inverse kinematics function and moves the robot arm a single step in each iteration of the task space trajectory. This meant that the robot arm was being moved without reference to the entire joint space trajectory. By performing the inverse kinematics on the entire list of task space steps and then performing trajectory smoothing (low pass filter, damping) before the joint angles are published, the shaking could be eliminated from the robot arm movement.

.. code-block::
   
   needs code here

Moving a Brick
----
Pickplace allows for a start position and end position to be input into the function and for all the necessary arm and gripper movements to be executed to move the brick from start to finish with uniaxial motion. The reason for uniaxial motion is so that there is much less chance of the interfering with obstacles as the end effector is lifted above the construction before any horizontal movement. Note that the horizontal movement can be duoaxial.

.. code-block::
   
   needs code here    
    
Moving the Gripper
----
As opposed to using the gripper width topic in order to control the gripper fingers, the fingers were controlled indiviudally, incase that ever became necessary during the task.

.. code-block::
   
   needs code here
    
Debug Functions
----
A number of debugging functions were used in order to test various aspects of the code. 

The **"franka_test"** function comes from the example_joint_publisher.py script provided to us. It tests that the franka panda robot and simulation are working as well as ros (the topics and publishers for the robot arm and gripper). This was the the most basic test for functionality and further development.

The **"sequence"** function tests efficacy of the inverse kinematics solver. The function instructs the robot arm to move to a number (4) of positions to demonstrate that the inverse kinematic solver works irrespective of any trajectory planning. If the robot does not move through the positions but the "franka_test" fuction does work then the issue can be narrrowed to the inverse kinematics solver. The "sequence" function was hugely helpful in establishing that the inverse kinematics solver was sometimes returning unsatisfactory outputs (made the movement unstable and took long routes on occasion) and led to us realising that we were using an abritrary seed state (inital state) for the inverse kinematics as opposed to the current position.

The **"joint_move_test"** function test the trajcotry planning function. It is essentially the same as the "sequence" function except with a slower, smoother movement. A comparison of this and the "sequence" function is very helpful in demonstrating the efficacy of the trajectory planning.

The **"pick_brick"** function tests the robot arm picking up a single brick. As well as testing the for all the aforementioned functions, this funciton crucially also tests the Gazebo simulation physics and interactions between the brick and the grippers. The "pick_brick" function also was used to find the correct Cartesian end-effector orientation to be converted to a quaternion to input into the IK solver. This function was also used in order to determine issues.

.. code-block::
   
   needs code here
