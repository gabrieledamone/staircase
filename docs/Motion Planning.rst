====
Motion Planning and Redundancy Resolution
====

Limiting Motion
====
In order to help with redundancy resolution, the end effector orientation was kept the same (gripper facing downwards) throughout the entirety of the project (with the exception of the push function).

Linear Interpolation
====
In order to control the speed of the robot arm and produce a smoother motion, linear interpolation was implemented with the calculated path (straight line from start to end position) discretised with an arbitrary number of steps. This was then subsequently improved such that the number of steps depended on the distance of the path, improving the consistency of the motion. Whilst this improved the speed, there was still a considerable amount of jerkiness and apparent overshooting in the robot motion. It became clear that the issue was not with the discretised path in Cartesian space, but with the joint angles returned by the inverse kinematics solver for each step. We hypothesised that error margain setting in the IK solver we were using was too small, however reducing it significantly caused no solution to be found within the timeout period, returning None. Increasing the timeout period significantly would cause the interpolation function to slow down considerably. In the end, we decided to perform post-processing on the joint angles returned by the solver and then publish them to the robot rather than publishing each step as it is calculated.

Joint Interpolation
====
In order to further smooth the movement of the arm, interpolating in joint space as well as task space was attempted, giving further control of the robot speed. The joint interpolator, however, did not reduce the high frequency noise (shaking) of the robot arm in the simulation. In theory, even if the task space output from the inverse kinematics resulted in overshooting (refer to figure of joint angles against time), the joint angle interoplator should have reduced the frequency of the shaking, however using the function appeared to slow down the simulation to the extent that it no longer functioned properly (slowing and peculiar movement).

Trapezium Velocity Profile
====

Trajectory Smoothing - Graph of Joint Angles against time
----

In order to perform some smoothing on the joint angles, initally a simple low-pass filter was tried whereby::
            
            a[i] = alpha*a[i] + (1-alpha)*a[i-1] 

.. note::

where a[i] is the angle step and 0 < alpha < 1. However this was not sufficient to remove some of the larger deviations in the curve (for lwo values of alpha) without affecting the intended movements (for higher values of alpha).

A more complex and potent curve smoothing was required and so we attempted to use savgol_filter function from the scipy.signal library. Smoothing the discretised joint angle positions with a Savitzky-Golay filter can remove the high frequency noise from the signal by fitting sections of the curve to the closest polynomial. In the end, a simple implementation of the function did not give intended results, however more testing with the function parameters could make this very potent for our scenario.
