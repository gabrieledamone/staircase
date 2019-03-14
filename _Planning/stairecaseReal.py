!/usr/bin/env python

"""
Name: staircaseReal.py
"""

import rospy
import math
import numpy as np
import map as map
import copy as copy
from math import cos, sin, atan2, pi
from std_msgs.msg import Float64MultiArray, MultiArrayDimension, Float64
from sensor_msgs.msg import JointState
from gazebo_msgs.msg import LinkStates
from trac_ik_python.trac_ik import IK
from tf.transformations import euler_from_quaternion, quaternion_from_euler

global position
global ef_pos
global q1

q1 = quaternion_from_euler(-3.141, 0, 2.35) # gripper facing downwards


## Data update. See Use of ROS in the documentation.
def callback(data): # retrieves the joint angle positions from the rostopic subscriber
    global position
    position = data.position
    # print position

def ef_pos_get(data): # retrieves the end_effector position from the rostopic subscriber
    global ef_pos
    ef_pos = data.pose[8].position


## Debug functions. See Key Functions in the documentation.
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

def joint_move_test(publishers): # testing the joint_move function. See Source Code in documentation
    print("Testing Joint Move Function")
    joint_move(publishers, [0.5, 0, 0.8])
    rospy.sleep(5)
    joint_move(publishers, [0.5, 0, 0.3])

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


## Solvers
def round_down(n, decimals=0):
    multiplier = 10 ** decimals
    return math.floor(n * multiplier) / multiplier

def ik_solver(X, Y, Z, QX, QY, QZ, QW): # trac_ik inverse kinematics solver. See IK Solver in documentation
    urdf_str = rospy.get_param('/robot_description')
    # print urdf_str
    ik_sol = IK("panda_link0","panda_link7",urdf_string=urdf_str)
    # print ik_sol.link_names

    global position
    seed_state = position[2:9]

    lower_bound, upper_bound = ik_sol.get_joint_limits()
    # print upper_bound
    # print lower_bound
    ik_sol.set_joint_limits(lower_bound, upper_bound)

    return ik_sol.get_ik(seed_state,
                X, Y, Z,  # X, Y, Z
                QX, QY, QZ, QW)  # QX, QY, QZ, QW

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

def joint_interpolator(publishers, current_angles, end_angles): # See Motion Planning in documentation
    inter = 100
    steps = []
    for i in range(7):
        steps.append(np.linspace(current_angles[i], end_angles[i], num=inter))
    for j in range(inter):
        for i in range(7):
            publishers[i].publish(round(steps[i][j],3))

def linear_interpolation(start_pos, end_pos): # See Motion Planning in documentation
    inter = 100
    steps = []
    for i in range(3):
        steps.append(np.linspace(start_pos[i], end_pos[i], num=inter))
    return steps

def apply_trapezoid_vel(path, acceleration=1, max_speed=1):
        """
        Takes a path (currently only a start/end point (straight line), and returns a discretised
        trajectory of the path controlled by a trapezium velocity profile generated by the input
        parameters.
        :param path: List of two points in 3D space.
        :param acceleration: Acceleration and deceleration of trapezium profile.
        :param max_speed: Target maximum speed of the trapezium profile.
        :return: Trajectory as numpy array.
        """
        # set rate of message sending:  0.001 sec == dt == 1kHz  NOTE THIS IS GLOBALLY SET
        dt = 0.005
        # set acceleration, start with 0.1 (may need to reduce)  NOTE THIS IS GLOBALLY SET
        acc = acceleration  # max 1.0
        # set target travel speed for motion
        target_speed = max_speed  # max 1.0

        # discretise using a max unit of:  target_speed * dt
        # ideally, we use a dx of: acc * dt**2
        dx = acc * dt**2  # this is the ideal delta value

        # if debug:
        #     print("delta displacement (mm): ", dx * 1000)

        lop = 0
        while lop == 0:
            # SMOOTHING HAPPENS HERE
            # if smoothing is going to happen it MUST keep consistent delta displacement
            # corner = 0.05  # in meters
            # steps = int(corner * 2 / dx)
            # print("Steps: ", steps)
            # smooth_path = planner.smooth_corners(dis_path, size_of_corner=steps, passes=6)
            smooth_path = discretise_path(path, dx) # disable smoothing

            # find the length of the new path
            lop = length_of_path(smooth_path)
            # if debug:
            #     print("LOP: ", lop)
            if lop == 0:
                print("Length of path is zero, adding indistinguishable offset.")
                path[1] = [c+0.000001 for c in path[1]]

        # check if the length of path is < ( speed**2/acc )
        # this means that the length of the path is too short to accelerate all the way to the
        # target speed so we scale it down to keep a triangular profile
        minimum_path_length = target_speed ** 2 / acc
        # if debug:
        #     print("Minimum path length: ", minimum_path_length)
        if lop < minimum_path_length:
            # if the length is less we need to reduce target_speed
            # if debug:
            #     print("Path length is too short.")
            old_speed = target_speed
            target_speed = np.sqrt(lop * acc)
            # if debug:
            #     print("Target speed changed from: ", old_speed, ", to: ", target_speed)

            # assert new target_speed is less than old for safety reasons
            assert (target_speed <= old_speed)

            # we have confirmed the length of the path is long enough for our target speed
            # if debug:
            #     print("Path length ok")

        # we now need to create the speed profile graph and define its parameters

        # find t for acceleration and deceleration
        end_stage_t = target_speed / acc
        # find path distance for acc and dec
        end_stage_displacement = end_stage_t * target_speed / 2
        #print("Acc/dec time: ", end_stage_t)

        # find displacement for constant speed section of motion
        mid_stage_displacement = lop - 2 * end_stage_displacement
        # find t for const speed section
        mid_stage_t = mid_stage_displacement / target_speed

        # find total time
        total_time = end_stage_t * 2 + mid_stage_t
        # if debug:
        #     print("total time: ", total_time)

        # create a time list using 0->T in steps of dt
        time_list = np.arange(start=0, stop=total_time, step=dt)
        np.reshape(time_list, (np.shape(time_list)[0], 1))

        # sample speed graph to create list to go with time list
        speed_values = []
        c = (0 - (-acc) * time_list[-1])
        for t in time_list:
            if t <= end_stage_t:
                # acceleration period
                speed_values.append(acc * t)

            elif t >= end_stage_t + mid_stage_t:
                # deceleration stage
                speed_values.append(-acc * t + c)

            elif t > end_stage_t:
                # constant speed at target speed
                speed_values.append(target_speed)

        # sample path using speed list
        # noinspection PyUnboundLocalVariable
        trajectory = np.hstack((smooth_path[0, :], speed_values[0]))  # send intermediate points
        smooth_path_idx = 0

        for i in range(1, len(speed_values)):
            samples = int(np.rint(speed_values[i] * dt / dx))
            smooth_path_idx += samples
            if smooth_path_idx > len(smooth_path) - 1:
                smooth_path_idx = len(smooth_path) - 1
            new_marker = np.hstack((smooth_path[smooth_path_idx], speed_values[i]))
            trajectory = np.vstack((trajectory, new_marker))
        return trajectory

def length_of_path(path):
        """
        Calculates the length of a path stored as an array of (n x 3).
        :param path: List (length n) of list (length 3) points.
        :return: The total length of the path in 3D space.
        """
        length = 0
        for i in range(len(path) - 1):
            point_a = path[i]
            point_b = path[i + 1]
            length += np.sqrt((point_b[0] - point_a[0]) ** 2 + (point_b[1] - point_a[1]) ** 2
                              + (point_b[2] - point_a[2]) ** 2)
        return length

def discretise(point_1, point_2, dx):
        """
        Takes a straight line and divides it into smaller defined length segments.
        :param point_1: First point in 3D space
        :param point_2: Second point in 3D space
        :param dx: Distance between points in discretised line.
        :return: Numpy array of discretised line.
        """
        # create vector from point_1 to point_2
        vector = [point_2[0] - point_1[0], point_2[1] - point_1[1], point_2[2] - point_1[2]]
        # noinspection PyUnresolvedReferences
        distance = np.sqrt(sum(i ** 2 for i in vector))

        # number of points on line
        i = int(distance / dx)

        # discretise by creating new 1d array
        line_x = np.linspace(point_1[0], point_2[0], i)
        line_y = np.linspace(point_1[1], point_2[1], i)
        line_z = np.linspace(point_1[2], point_2[2], i)
        line = np.array(np.transpose(np.vstack((line_x, line_y, line_z))))
        return line

def discretise_path( move, dx):
        """
        Discretise a moves path using object defined dx for unit.
        :param move: List of points path goes through.
        :param dx: Displacement between two points on the target discretised path.
        :return: Discretised path as numpy array.
        """
        move_discrete = []
        # iterate through move segments, discretise and join them
        for seg_idx in range(len(move) - 1):
            current_segment = discretise(move[seg_idx], move[seg_idx + 1], dx)

            # print(current_segment)
            # we add our discretised segment to our move
            if seg_idx > 0:
                # if the end of our current move is the same position as the start of our new
                # segment then we only want to add the list from the second point onwards
                if move_discrete[-1][0] == current_segment[0][0]:
                    # noinspection PyUnresolvedReferences
                    move_discrete = np.concatenate((move_discrete, current_segment[1:]))
                else:
                    # noinspection PyUnresolvedReferences
                    move_discrete = np.concatenate((move_discrete, current_segment))

            else:  # on first iteration, we store our segment directly
                move_discrete = current_segment
        return move_discrete


## Movements
def grip_move(grip_pos, grip_pub, f1, f2): # See Key Functions in the documentation.
    grip_pos.data = [f1, f2]
    grip_pub.publish(grip_pos)
    return

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

def push(publishers, grip_pos, grip_pub):
    q_push = quaternion_from_euler(-1.57,-0.785, 0)
    #print q1
    initial = [-0.0027898559799117706, -0.4938102538628373, 0.011231754474766653, -2.4278711125230714, -0.014718553972133286, 1.889487912176289, -2.300243077342502]
    for i in range(7):
        publishers[i].publish(initial[i])
    rospy.sleep(5)

    rospy.loginfo("Pushing Action")
    joint_move(publishers, [0.4, 0.0, 0.064], q_push)
    #rospy.sleep(2)

    rospy.loginfo("Opening Gripper")
    grip_move(grip_pos, grip_pub, 1, 1)
    rospy.sleep(2)

    rospy.loginfo("Pushing Action")
    joint_move(publishers, [0.4, 0.25, 0.064], q_push)
    rospy.sleep(2)

    rospy.loginfo("Back to centre")
    joint_move(publishers, [0.4, 0.0, 0.064], q_push)


if __name__ == '__main__':
    global position
    rospy.init_node('sc_pub')

    # Arm Initialise
    arm_pubs = [rospy.Publisher('/franka/joint{}_position_controller/command'.format(i), Float64, queue_size=1) for i in range(1, 8)]
    arm_pos = rospy.Subscriber("/joint_states", JointState, callback, queue_size=1)
    ef_pos = rospy.Subscriber("/gazebo/link_states", LinkStates, ef_pos_get, queue_size=10)

    # Gripper Initialise
    grip_pub = rospy.Publisher('/franka/gripper_position_controller/command', Float64MultiArray, queue_size=1)
    #gripper_width = rospy.Subscriber("/franka/gripper_width", Float64, callback, queue_size=1)
    grip_pos = Float64MultiArray()
    grip_pos.layout.dim = [MultiArrayDimension('', 2, 1)]

    rospy.sleep(5)

    start = rospy.Time.now()
    rate = rospy.Rate(10)

    initial = [0, 0, 0, -0.5, 0, 0.5, 0.75]
    for i in range(7):
        arm_pubs[i].publish(initial[i])

    # Debug
    # franka_test(start, arm_pubs, initial, grip_pos, grip_pub)
    # sequence(arm_pubs)
    # pick_brick(arm_pubs, grip_pos, grip_pub)
    # joint_move_test(arm_pubs)


    ### DOING IT

    height = int(input("Height of Staircase: ")) # ask for height of staircase
    # print(height)
    print("Building staircase of height " + str(height)) # output for transparency
    # if height >= 5: print("Sorry our robot is lazy today") # lazy
    # heightAmount = {1: 1,2: 3,3: 6,4: 10,5: 15} # define amount of bricks necessary dependent on height
    # brickNums = heightAmount(height) # count amount of bricks necessary = length of location array]

    locationDestinationOptions = [map.one, map.two, map.three, map.four, map.five] # load options array of height station maps, order of bricks is order of pick up: right to left view from top

    # heightMap = [[],[],[],[]]

    locationDestinationMap = locationDestinationOptions[height-1] # read location/destination array from options array from height with locations in order of all bricks (according to logic) and where they need to go
    # run function which uses locationDestinationMap to place bricks in Gazebo
    station = [i[0] for i in locationDestinationMap] # define station array listing starting locations for bricks
    destination = [j[1] for j in locationDestinationMap] # define destination array for destination locations on where bricks need to go
    # define actual array reading actual locations of bricks for the robot to avoid obstacles

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

    push(arm_pubs, grip_pos, grip_pub) # pushing things into a real staircase
    # staircase(arm_pubs, grip_pos, grip_pub)
