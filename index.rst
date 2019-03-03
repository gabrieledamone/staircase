**********************************
DE3-ROB1 CHESS Group Documentation
**********************************

About
=====

This is the documentation for the group STAIRCASE Project for the Robotics 1 module in Design Engineering, Imperial College London, in March 2019.

The project is hosted on GitHub: https://github.com/gabrieledamone/staircase

The Authors
-----------

- Gabriele D'Amone (gabriele.damone16@imperial.ac.uk)
- SeungHui Huh 
- YongXuam Li
- Huyang Liu 
- Fred Eric Macher 
- William Kwasi Pepera 
- Federico Tiersen 



Summary
=======

The goal of this project was to create a fully automated staircase-builder robot by applying code to a FRANKA Panda arm. The project was written in Python and ROS was used to interface with FRANKA.

...was used..

Future improvements to the project could include...

Popular Links
=============

* `Using Python to control Franka (without ROS)`_.
* `Using Python to control Franka with ROS topics`_.
* `Converting points between reference frames`_.

.. _`Using Python to control Franka (without ROS)`: operating.html
.. _`Using Python to control Franka with ROS topics`: franka_ros.html
.. _`Converting points between reference frames`: calibration.html

Using Test Scripts
==================

Throughout our project we used test scripts. These can be seen in the ``tests`` folder. To run these tests properly (e.g. ``test_camera.py``) you should type the following into the terminal::

  cd DE3-ROB1-CHESS/
  python -m tests.test_camera.py

This is to ensure relative imports work properly, as every import is relative to the project level directory.

Contents
========

.. toctree::
   :maxdepth: 2
   :caption: Working with FRANKA Emika

   franka
   workstation
   operating

.. toctree::
   :maxdepth: 2
   :caption: Project Development

   camera
   calibration
   perception
   chess-engine
   motion
   controller

.. toctree::
   :maxdepth: 2
   :caption: Project Management

   project-proposal
   project-plan

.. toctree::
   :maxdepth: 2
   :caption: Appendix

   resources
   ground-rules

Indices and tables
==================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`


## Contents
- Setting up Franka Emika Panda & Workstation
- Project Specifications & Requirements
- Using MoveIt with the RViz & Gazebo Implementation
	- Introduction to MoveIt
	- Running the Pick and Place demo in RViz
	- Error when executing the RViz trajectory in Gazebo
	- Debugging
	- Exploring possible solutions / Giving up on MoveIt
- IK Solver
	- Picking the Brick
		- Gripper not wide enough to pick brick 
		- The brick had to be turned on the side, but the centre of gravity was not in the centre and the brick kept falling
		- We changed the centre of gravity and managed to pick the brick correctly
		- The tutors made the gripper wider, we managed to pick up the brick from the horizontal position
		- Some bricks were sliding away, therefore we increased the friction of the brick
- Motion planning 
- Making the robot fully Autonomous
	- User input for number of stairs
- Robustness
	- How do we solve misaligned object orientation? 
- Error Detection
	- Failing to grasp triggers a regrasping attempt
- Ambition
	- Human level performance, with one arm!
- Outstanding features
	- Pushing mechanism
- Speed
	- Trapezium Velocity Profile
- Motion Planning
- Redundancy Resolution
	- Fixing elbow or orientation of end effector?
- Design of initial workspace
	- Feeding the brick manually
	- Positioning strategically the stock bricks


