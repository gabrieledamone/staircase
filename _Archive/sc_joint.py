#!/usr/bin/env python
"""
"""
import rospy
import math
import numpy as np
from math import cos, sin, atan2, pi
from std_msgs.msg import Float64MultiArray, MultiArrayDimension, Float64
from sensor_msgs.msg import JointState
from trac_ik_python.trac_ik import IK
from tf.transformations import euler_from_quaternion, quaternion_from_euler

global position

def callback(data):
    global position
    position = data.position
    #print position

# Debug Functions
def franka_test(start, publishers, initial, grip_pos, grip_pub):
    rospy.loginfo("Testing Franka Movement")
    while not rospy.is_shutdown():
        elapsed = rospy.Time.now() - start
        delta_angle = math.pi / 16 * (math.cos(math.pi / 5 * elapsed.to_sec()))

        for i in range(7):
            publishers[i].publish(initial[i] + delta_angle)
            #print initial[i]+delta_angle

        grip_move(grip_pos,grip_pub, delta_angle*10, delta_angle*10)
        print delta_angle*10
        #grip_pos.data = [delta_angle*10, delta_angle*10]
        #grip_pub.publish(grip_pos)

        rate.sleep()

def sequence(publishers):
    rospy.loginfo("Testing ik solver")
    q1 = quaternion_from_euler(-3.14, 0.0, 2.3)
    pos1 = ik_solver(0.1, 0.1, 0.5, q1[0], q1[1], q1[2], q1[3])
    pos2 = ik_solver(0.3, 0.1, 0.5, q1[0], q1[1], q1[2], q1[3])
    pos3 = ik_solver(0.5, 0.1, 0.5, q1[0], q1[1], q1[2], q1[3])
    pos4 = ik_solver(0.7, 0.1, 0.5, q1[0], q1[1], q1[2], q1[3])

    print pos1, pos2, pos3, pos4

    routine = [pos1, pos2, pos3, pos4]
    for j in range (len(routine)):
        if routine[j] is None:
            continue
        for i in range(7):
            publishers[i].publish(routine[j][i])
        rospy.sleep(0.1)

#Solvers
def ik_solver(X, Y, Z, QX, QY, QZ, QW):
    global position
    urdf_str = rospy.get_param('/robot_description')
    #print urdf_str
    ik_sol = IK("panda_link0","panda_link7",urdf_string=urdf_str)
    #print ik_sol.link_names
    seed_state = position[2:9]

    lower_bound, upper_bound = ik_sol.get_joint_limits()
    #print upper_bound
    #print lower_bound
    ik_sol.set_joint_limits(lower_bound, upper_bound)

    return ik_sol.get_ik(seed_state,
                X, Y, Z,  # X, Y, Z
                QX, QY, QZ, QW)  # QX, QY, QZ, QW

def joint_move(publishers, pos):
    q1 = quaternion_from_euler(-3.14, 0.0, 2.3) # gripper facing downwards
    steps = 100
    for j in range(steps):
        step = ik_solver(pos[0]*(j/steps), pos[1]*(j/steps), pos[2]*(j/steps), q1[0], q1[1], q1[2], q1[3])
        print step
        if step is None:
            continue
        for i in range(7):
            publishers[i].publish(step[i])


# Movements
def grip_move(grip_pos, grip_pub, f1, f2):
    grip_pos.data = [f1, f2]
    grip_pub.publish(grip_pos)
    return

def pick_brick(publishers, grip_pos, grip_pub): # picks up brick placed at x=0.4, y=0, z=0, roll, pitch, yaw=0
    q1 = quaternion_from_euler(-3.14, 0.0, 2.3) # gripper facing downwards

    #print q1
    initial = [-0.0027898559799117706, -0.4938102538628373, 0.011231754474766653, -2.4278711125230714, -0.014718553972133286, 1.889487912176289, -2.300243077342502]
    for i in range(7):
        publishers[i].publish(initial[i])
    rospy.sleep(5)

    step1 = ik_solver(0.4, 0.0, 0.6, q1[0], q1[1], q1[2], q1[3])
    step2 = ik_solver(0.4, 0.0, 0.28, q1[0], q1[1], q1[2], q1[3])

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

def staircase_1(publishers, grip_pos, grip_pub):
    # gripper facing downwards
    qy1 = quaternion_from_euler(-3.14, 0.0, 2.3) # gripper parallel y1
    qy2 = quaternion_from_euler(-3.14, 0.0, 5.44) # gripper parallel y2
    qx1 = quaternion_from_euler(-3.14, 0.0, 3.87) # gripper parallel x1
    qx2 = quaternion_from_euler(-3.14, 0.0, 0.728) # gripper parallel x2

    bph = 0.29 # brick pick height
    blh = 0.6 # brick lift height   

    brick_num = 3
    brick_start = [[0.6, -0.2, bph],[0.6, -0.0, bph],[0.6, 0.2, bph]]
    brick_end = [[0.4, 0.0, bph],[0.4, 0.1, bph],[0.4, 0.1, bph+0.09]]

    for i in range(brick_num+1):
        if i == 0:
            initial = [-0.014534011874288488, -0.4938412735927953, 0.022786422955764607, -2.427866515018759, -0.009623357439607183, 1.8894895676194317, -2.304773463241591]
            for j in range(7):
                publishers[j].publish(initial[j])
            grip_move(grip_pos, grip_pub, 1, 1)
            rospy.sleep(5)
            continue

        print "moving brick", i
        grip_move(grip_pos, grip_pub, 1, 1)
        rospy.sleep(5)

        # Moving Brick
        joint_move(publishers, brick_start[i-1])
        joint_move(publishers, brick_end[i-1])

        grip_move(grip_pos, grip_pub, 1, 1)
   
    

if __name__ == '__main__':
    rospy.init_node('sc_pub')

    # Arm Initialise
    arm_pubs = [rospy.Publisher('/franka/joint{}_position_controller/command'.format(i), Float64, queue_size=1) for i in range(1, 8)]
    arm_pos = rospy.Subscriber("/joint_states", JointState, callback, queue_size=1)
    global position

    # Gripper Initialise
    grip_pub = rospy.Publisher('/franka/gripper_position_controller/command', Float64MultiArray, queue_size=1)
    #gripper_width = rospy.Subscriber("/franka/gripper_width", Float64, callback, queue_size=1)
    grip_pos = Float64MultiArray()
    grip_pos.layout.dim = [MultiArrayDimension('', 2, 1)]

    rospy.sleep(5)

    #Set to initial position
    #rospy.loginfo("Setting initial pose.")
    '''
    initial = [0, 0, 0, -0.5, 0, 0.5, 0.75]
    for i in range(7):
        arm_pubs[i].publish(initial[i])

    rospy.sleep(5)
    '''

    start = rospy.Time.now()
    rate = rospy.Rate(10)

    # Debug
    #franka_test(start, arm_pubs, initial, grip_pos, grip_pub)
    sequence(arm_pubs)

    #pick_brick(arm_pubs, grip_pos, grip_pub)
    #staircase_1(arm_pubs, grip_pos, grip_pub)