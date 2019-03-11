**********
Source Code
**********

Joint Move
===========
The core joint planning function used was the "joint_move" function. Inputting the joint publishers of the robot arm and the desired end effector position moves the robot arm from its current end effector position to the desired position using the specified trajectory planning. A key improvement would be to give the user a choice of trajectory planning alogrithms (linear, trigonometric, polynomial, trapezoid velocity profile etc.) for different situations.

Currently, the joint move function employs the inverse kinematics function and moves the robot arm a single step in each iteration of the task space trajectory. This meant that the robot arm was being moved without reference to the entire joint space trajectory. By performing the inverse kinematics on the entire list of task space steps and then performing trajectory smoothing (low pass filter, damping) before the joint angles are published, the shaking could be eliminated from the robot arm movement.

def joint_move(publishers, end_pos): # movement smoothing
    
    # input the joing publishers list and the end position in the form [x, y, z]
    
    global ef_pos # end effector position from rostopic
    global position # joint angle positions from rostopic [2:9] fo joint positions
    global q1

    current_pos = [ef_pos.x, ef_pos.y, ef_pos.z]
    current_angles = np.array(position[2:9])

    # end effector position interpolation

    trajectory = apply_trapezoid_vel([current_pos, end_pos], acceleration=10, max_speed=1) # trapezium speed profile
    # trajectory = linear_interpolation(current_pos, end_pos)

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
            
        for i in range(7):
            publishers[i].publish(step[i])
