====
Project Proposal
====

STAIRCASE - Robotic Arm Building Bricks Staircase
====

The goal of this project is to program a robotic arm to build a **bricks staircase**. The robot used will most likely be the ``FRANKA Emika`` robotic arm (nicknamed *Panda*) as it is more precise in positioning and grasping than ``DE NIRO``. The outcome of the project will be demonstrated by performing a fully autonomous building of a staircase. 

You can see what the robot looks like here: http://www.imperial.ac.uk/robot-intelligence/robots/franka-emika/

The project includes:

1. **Forward and Inverse Kinematics:** To use the 7-dof kinematic model for Cartesian-to-Joint space mapping
2. **Redundancy Resolution:** To resolve the 7-dof arm configuration while following a 6-dof hand pose trajectory
3. **Motion Planning:** Using OMPL/MoveIt or other motion planning libraries to generate viable collision-free trajectories for executing the movements
4. **Motion Control:** To tune a Cartesian impedance controller or other for fast and smooth motion of the arm
5. **Hand control:** To control the grasping with the 2 fingers of the hand

**Equipment:** ``FRANKA Emika`` (Panda) and bricks.
