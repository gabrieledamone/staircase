**********
Motion Planning
**********

Trajectory Planning
===========
https://www.researchgate.net/publication/288386128_Picking_Robot_Arm_Trajectory_Planning_Method


https://www.ncbi.nlm.nih.gov/pmc/articles/PMC6165411/
Whilst linear interpolation 

https://en.wikipedia.org/wiki/Savitzky–Golay_filter


Linear Interpolation
===========


Joint Interpolation
===========
In order to further smooth the movement of the arm, interpolating in joint space as well as task space was attempted, giving further control of the robot speed. The joint interpolator, however, did not reduce the high frequency noise (shaking) of the robot arm in the simulation. In theory, even if the task space output from the inverse kinematics resulted in overshooting (refer to figure of joint angles against time), the joint angle interoplator should have reduced the frequency of the shaking, however using the function appeared to slow down the simulation to the extent that it no longer functioned properly (slowing and peculiar movement).

def joint_interpolator(publishers, current_angles, end_angles):
    inter = 100
    steps = []
    for i in range(7):
        steps.append(np.linspace(current_angles[i], end_angles[i], num=inter))
    for j in range(inter):
        for i in range(7):
            publishers[i].publish(round(steps[i][j],3))
            
        
