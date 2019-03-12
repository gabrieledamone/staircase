**********
IK Solver
**********

https://bitbucket.org/traclabs/trac_ik/src/HEAD/trac_ik_python/

The inverse kinematics solver used was trac_ik. It has a python wrapper which was used in order to include it in our solution. The solver uses quaternions as the inputs for the end effector orientation and Cartesian coordinates for the position. 

The trac_ik solver returns the first valid solution returned, however the timeout period, the error margin between the input end effector position and the resultant one (epsilon) and solver type can be configured. We found that the Distance solver type slightly increased the accuracy of the solver with regards to finding the joint angles that constituted the shortest path from start to goal. As well as this, the ability to set joint limits for the IK solver helped to constrain some of the motion of the robot and slightly increased solution speed (noticed during the interpolation where there are many calculations).

Initially, the function was not working as intended because our seed state was 0 rad for every joint and so the solver was returning the first solution as calculated from that arbitrary state. Subsequently, the function was changed such that it uses the current joint angles from the joint_states rostopic, written to the global variable "position".

KDL vs track_ik
===========
"Specifically, KDL's convergence algorithms are based on Newton's method, which does not work well in the presence of joint limits --- common for many robotic platforms. TRAC-IK concurrently runs two IK implementations. One is a simple extension to KDL's Newton-based convergence algorithm that detects and mitigates local minima due to joint limits by random jumps. The second is an SQP (Sequential Quadratic Programming) nonlinear optimization approach which uses quasi-Newton methods that better handle joint limits. By default, the IK search returns immediately when either of these algorithms converges to an answer. Secondary constraints of distance and manipulability are also provided in order to receive back the "best" IK solution."
