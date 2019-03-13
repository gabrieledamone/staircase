====
Code Explanation
====
Let's have a look at our final code: it's main *objectives*, *structure* and *key functions* within it.

Objectives
====
As discussed previously we have designed the functionalites of the final code to be programmed by **individual groups** and then be put together at a later date to produce the **result** we have today.
The below features are *key* in our programming execution and what we understand as *milestones* in our work.

Picking & Placing
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

.. code-block::

    height = int(input("Height of Staircase: ")) # ask for height of staircase
    # print(height)
    print("Building staircase of height " + str(height)) # output for transparency
    # if height >= 5: print("Sorry our robot is lazy today") # lazy
    # heightAmount = {1: 1,2: 3,3: 6,4: 10,5: 15} # define amount of bricks necessary dependent on height
    # brickNums = heightAmount(height) # count amount of bricks necessary = length of location array]

2. Then are loaded the relevant ``workspace maps`` for this specific ``height`` ie. the **initial** and **final** positions of the bricks.

.. code-block::

   locationDestinationOptions = [map.one, map.two, map.three, map.four, map.five] # load options array of height station maps,      order of bricks is order of pick up: right to left view from top
    # heightMap = [[],[],[],[]]
    locationDestinationMap = locationDestinationOptions[height-1] # read location/destination array from options array from height with locations in order of all bricks (according to logic) and where they need to go
    # run function which uses locationDestinationMap to place bricks in Gazebo
    station = [i[0] for i in locationDestinationMap] # define station array listing starting locations for bricks
    destination = [j[1] for j in locationDestinationMap] # define destination array for destination locations on where bricks need to go
    # define actual array reading actual locations of bricks for the robot to avoid obstacles

3. Finally a ``while loop`` is run which looks like this:

.. code-block::

    while True: # while loop to place all functions
        tracker = copy.deepcopy(locationDestinationMap) # tracker to exit while loop when all bricks are placed
        counter = 0 # initialise counter
        for k in locationDestinationMap:
            brickStation = station[counter] # station reading
            brickDestination = destination[counter] # destination reading
            pickPlace(arm_pubs, grip_pos, grip_pub, brickStation, brickDestination)
            tracker.pop(0) # removing the brick we just worked with
            counter += 1 # keep counting
            if not tracker: # exit once done
                break
        break
    push(publishers, grip_pos, grip_pub) # pushing things into a real staircase
    # staircase(arm_pubs, grip_pos, grip_pub)

Let's now have a look at the functions called to build the staircase in the next section: ``pickPlace`` and ``push``.

Below the code again in all it's glory:

.. literalinclude:: ../_Planning/staircaseReal.py
   :lines: 485-496


Key Functions
====

Going further from our objectives we came to define key functions in our codes functioning that we are visiting below.

Joint Move
----
The core joint planning function used was the ``joint_move`` function. Inputting the joint publishers of the robot arm and the desired end effector position moves the robot arm from its **current** end effector position to the **desired** position using the specified *trajectory planning*. A key improvement would be to give the user a choice of trajectory planning alogrithms (linear, trigonometric, polynomial, trapezoid velocity profile etc.) for different situations.

Currently, the ``joint_move`` function employs the inverse kinematics function and moves the robot arm a single step in each iteration of the task space trajectory. This meant that the robot arm was being moved without reference to the entire joint space trajectory. By performing the inverse kinematics on the entire list of task space steps and then performing trajectory smoothing (low pass filter, damping) before the joint angles are published, the shaking could be eliminated from the robot arm movement.

.. code-block::
   
   def joint_move(publishers, end_pos, motion_planner='trapezium'): # movement smoothing. See Source Code in documentation
    
    # input the joing publishers list and the end position in the form [x, y, z]

    global ef_pos # end effector position from rostopic
    global position # joint angle positions from rostopic [2:9] fo joint positions
    global q1
    # print end_pos

    current_pos = [ef_pos.x, ef_pos.y, ef_pos.z]
    current_angles = np.array(position[2:9])

    # end effector position interpolation
    '''
    # No motion planning
    step = ik_solver(end_pos[0], end_pos[1], end_pos[2], q1[0], q1[1], q1[2], q1[3])
    for i in range(7):
        publishers[i].publish(step[i])
    '''

    if motion_planner == 'trapezium':
        trajectory = apply_trapezoid_vel([current_pos, end_pos], acceleration=10, max_speed=1) # trapezium speed profile
    elif motion_planner == 'linear':
        trajectory = linear_interpolation(current_pos, end_pos)

    for i in range(len(trajectory)):
        for j in range(3):
            trajectory[i][j] = round(trajectory[i][j], 2)

    for i in range(len(trajectory)):
        print trajectory[i]
        try:
            step = ik_solver(trajectory[i][0], trajectory[i][1], trajectory[i][2], q1[0], q1[1], q1[2], q1[3])
        except IndexError:
            continue
        if step is None:
            continue

        # joint_interpolator(publishers, current_angles, step)
        for i in range(7):
            publishers[i].publish(step[i])

Moving a Brick
----
``PickPlace`` allows for a **start** position and **end** position to be input into the function and for all the necessary *arm and gripper* movements to be executed to move the brick from start to finish with uniaxial motion. The reason for uniaxial motion is so that there is much less chance of the interfering with obstacles as the end effector is lifted above the construction before any horizontal movement. Note that the horizontal movement can be duoaxial.

.. code-block::
   
   def pickPlace(publishers, grip_pos, grip_pub, brick_start, brick_end): # picks up brick and places it at the end position. Function ends with end_effecttor above end position. See Key Functions in the documentation.
    raise_height = 0.8 # the height that the end effector will lift bricks to when moving
    sleep_time = 2 # amount of seconds between movements
    pbh = 0.243 # the height above the brick that the end effector must be to lift it

    joint_move(publishers, [brick_start[0], brick_start[1], raise_height]) # moving gripper above brick start
    rospy.sleep(sleep_time)
    grip_move(grip_pos, grip_pub, 1, 1) # opening grippers
    rospy.sleep(sleep_time)
    joint_move(publishers, [brick_start[0], brick_start[1], brick_start[2]+pbh]) # moving to pick brick height
    rospy.sleep(sleep_time)
    grip_move(grip_pos, grip_pub, -1, -1) # closing grippers
    rospy.sleep(sleep_time)
    joint_move(publishers, [brick_start[0], brick_start[1], raise_height]) # moving to brick raise height
    rospy.sleep(sleep_time)
    joint_move(publishers, [brick_end[0], brick_end[1], raise_height]) # moving to above brick destination
    rospy.sleep(sleep_time)
    joint_move(publishers, [brick_end[0], brick_end[1], brick_end[2]+pbh]) # moving to brick end position
    rospy.sleep(sleep_time)
    grip_move(grip_pos, grip_pub, 1, 1) # opening grippers
    rospy.sleep(sleep_time)
    joint_move(publishers, [brick_end[0], brick_end[1], raise_height]) # moving back up in preparation for next brick    
    
Moving the Gripper
----
As opposed to using the gripper width topic in order to control the gripper fingers, the fingers were controlled indiviudally, incase that ever became necessary during the task.

.. code-block::
   
   def grip_move(grip_pos, grip_pub, f1, f2): # See Key Functions in the documentation.
    grip_pos.data = [f1, f2]
    grip_pub.publish(grip_pos)
    return
   
Pushing
----
The ``push`` function was defined to push horizontally all the bricks together at the end of the staircase building.

.. code-block::
   
   def push(publishers, grip_pos, grip_pub):
    q1 = quaternion_from_euler(-1.57,-0.785, 0)
    # print q1
    initial = [-0.0027898559799117706, -0.4938102538628373, 0.011231754474766653, -2.4278711125230714, -0.014718553972133286, 1.889487912176289, -2.300243077342502]
    for i in range(7):
        publishers[i].publish(initial[i])
    rospy.sleep(5)

    step1 = ik_solver(0.4, 0.2, 0.1, q1[0], q1[1], q1[2], q1[3])
    # step2 = ik_solver(0.4, 0.5, 0.2, q1[0], q1[1], q1[2], q1[3])

    rospy.loginfo("Moving Gripper in position")
    for i in range(7):
        publishers[i].publish(step1[i])
    rospy.sleep(2)

    rospy.loginfo("Opening Gripper")
    grip_move(grip_pos, grip_pub, 1, 1)
    rospy.sleep(2)


    rospy.loginfo("Pushing Action")
   
    joint_move_push(publishers, [0.4, 0.2, 0.1], [0.4, 0.5, 0.1])
    rospy.sleep(2)
    
Debug Functions
----
A number of debugging functions were used in order to test various aspects of the code. 

The ``franka_test`` function comes from the *example_joint_publisher.py* script provided to us. It tests that the franka panda robot and simulation are working as well as ros (the topics and publishers for the robot arm and gripper). This was the the most basic test for functionality and further development.

.. code-block::
   
   def franka_test(start, publishers, initial, grip_pos, grip_pub): # example joint publisher to test the franka movement is working. See Source Code in documentation
    rospy.loginfo("Testing Franka Movement")
    while not rospy.is_shutdown():
        elapsed = rospy.Time.now() - start
        delta_angle = math.pi / 16 * (math.cos(math.pi / 5 * elapsed.to_sec()))

        for i in range(7):
            publishers[i].publish(initial[i] + delta_angle)
            # print initial[i]+delta_angle

        grip_move(grip_pos,grip_pub, delta_angle*10, delta_angle*10)
        print delta_angle*10
        # grip_pos.data = [delta_angle*10, delta_angle*10]
        # grip_pub.publish(grip_pos)

        rate.sleep()

The ``sequence`` function tests efficacy of the inverse kinematics solver. The function instructs the robot arm to move to a number (4) of positions to demonstrate that the inverse kinematic solver works irrespective of any trajectory planning. If the robot does not move through the positions but the ``franka_test`` fuction does work then the issue can be narrrowed to the inverse kinematics solver. The ``sequence`` function was hugely helpful in establishing that the inverse kinematics solver was sometimes returning unsatisfactory outputs (made the movement unstable and took long routes on occasion) and led to us realising that we were using an abritrary seed state (inital state) for the inverse kinematics as opposed to the current position.

.. code-block::
   
   def sequence(publishers): # testing a sequence of positions to determine if the ik_solver function is giving the correct output. See Source Code in documentation
    rospy.loginfo("Testing ik solver")

    # positions in sequence
    pos1 = ik_solver(0.2, 0.1, 0.8, 0.0, 0.0, 0.0, 1.0)
    pos2 = ik_solver(0.3, 0.1, 0.8, 0.0, 0.0, 0.0, 1.0)
    pos3 = ik_solver(0.4, 0.1, 0.8, 0.0, 0.0, 0.0, 1.0)
    pos4 = ik_solver(0.5, 0.1, 0.8, 0.0, 0.0, 0.0, 1.0)

    print pos1, pos2, pos3, pos4

    routine = [pos1, pos2, pos3, pos4]
    for j in range (len(routine)):
        for i in range(7):
            publishers[i].publish(routine[j][i])
        rospy.sleep(5)

The ``joint_move_test`` function test the trajcotry planning function. It is essentially the same as the ``sequence`` function except with a slower, smoother movement. A comparison of this and the ``sequence`` function is very helpful in demonstrating the efficacy of the trajectory planning.

.. code-block::
   
   def joint_move_test(publishers): # testing the joint_move function. See Source Code in documentation
    print("Testing Joint Move Function")
    joint_move(publishers, [0.5, 0, 0.8])
    rospy.sleep(5)
    joint_move(publishers, [0.5, 0, 0.3])

The ``pick_brick`` function tests the robot arm picking up a single brick. As well as testing the for all the aforementioned functions, this funciton crucially also tests the Gazebo simulation physics and interactions between the brick and the grippers. The ``pick_brick`` function also was used to find the correct Cartesian end-effector orientation to be converted to a quaternion to input into the IK solver. This function was also used in order to determine issues.

.. code-block::
   
   def pick_brick(publishers, grip_pos, grip_pub): # picks up bring placed at x=0.4, y=0, z=0, roll, pitch, yaw=0
    q1 = quaternion_from_euler(-3.14, 0.0, 2.3) # gripper facing downwards

    # print q1
    initial = [-0.0027898559799117706, -0.4938102538628373, 0.011231754474766653, -2.4278711125230714, -0.014718553972133286, 1.889487912176289, -2.300243077342502]
    for i in range(7):
        publishers[i].publish(initial[i])
    rospy.sleep(5)

    step1 = ik_solver(0.4, 0.0, 0.6, q1[0], q1[1], q1[2], q1[3])
    step2 = ik_solver(0.4, 0.0, 0.25, q1[0], q1[1], q1[2], q1[3])

    rospy.loginfo("Moving Gripper above brick")
    for i in range(7):
        publishers[i].publish(step1[i])
    rospy.sleep(5)

    rospy.loginfo("Opening Gripper")
    grip_move(grip_pos, grip_pub, 1, 1)
    rospy.sleep(5)

    rospy.loginfo("Lowering Gripper")
    for i in range(7):
        publishers[i].publish(step2[i])
    rospy.sleep(5)

    rospy.loginfo("Closing Gripper")
    grip_move(grip_pos, grip_pub, -1, -1)
    rospy.sleep(5)

    rospy.loginfo("Lifting Brick")
    for i in range(7):
        publishers[i].publish(step1[i])
