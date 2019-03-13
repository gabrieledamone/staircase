**********
Trapezium Velocity Profile 
**********
    
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
