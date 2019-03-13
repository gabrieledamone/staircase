**********
Trapezium Velocity Profile 
**********
A typical way to control robotic joints is to use a trapezoidal speed profile, which is quite similar to the profiles you’d get by driving your car: acceleration, constant speed, and deceleration.

Imagine a vehicle running to its target. Instead of rushing and bumping into the end point, the vehicle must accelerate and keep running at a constant speed, and before reaching the goal, it will start decelerating and get to the end safely. That is the general idea of Trapezium Velocity Profile which we applied to the motion of Franka robot.

.. figure:: pictures/trapezium.png
    :align: center
    :figclass: align-center

Trapezium Velocity Profile is widely used in the industry, and the profile is constructed with two main input parameters: 
1.	The desired target speed of end-effector travel
2.	The acceleration of the end-effector.( it will be mirrored to create deceleration period)
The middle section is scaled according to the travel distance to keep the end-effector at a constant speed.

To implement the Trapezium Velocity Profile, firstly we need to discretise the path into a number of small, equally sized unit. The ideal size of each unit is dx which is calculated by: dx = acceleration * dt**2.

There are two case that we considered while applying the profile. The first case is that the length of the path is long enough for the end-effector to reach the target speed. The middle sectiond is scaled along the time axis accordingly.

Based on the two input parameters, the profile will be constructed::
    The end stage time is calculated: target_speed / acceleration
    The end stage displacement is calculated: end_stage_time * target_speed / 2
    The mid stage displacement is calculated: path_length - 2 * end_stage_displacement
    The mid stage time is calculated: mid_stage_displacement / target_speed
    The total time is calculated: end_stage_time * 2 + mid_stage_time

Then creating a time list using 0->Total time in steps of dt::

    time_list = np.arange(start=0, stop=total_time, step=dt)
    np.reshape(time_list, (np.shape(time_list)[0], 1))
        


.. note::
The dt variable is fixed by the control loop running the libfranka control loop. Therefore, it cannot be changed locally from 0.05 seconds in this trajectory generator.
