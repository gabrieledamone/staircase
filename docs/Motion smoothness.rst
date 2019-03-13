Trapezium Velocity Profile

Imagine a vehicle running to its target. Instead of rushing and bump into the end point, the vehicle must accelerate and keep running at a constant speed, and before reaching the goal, it will start decelerating and get to the end safely. That is the general idea of Trapezium Velocity Profile
 Which we applied to the motion of Franka robot.

Trapezium Velocity Profile is widely used in the industry, and the profile is constructed with two main input parameters: 
1.	The desired target speed of end-effector travel
2.	The acceleration the end-effector.( it will be mirrored to create deceleration period)
The middle section is scaled according to the travel distance to keep the end-effector at a constant speed.
 
To implement the Trapezium Velocity Profile, firstly we need to discretise the path into a number of small, equally sized unit. The ideal size of each unit is dx which is calculated by: dx = acc * dt**2  
