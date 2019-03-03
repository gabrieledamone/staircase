<h1 align="center">
	<img width="400" src="docs/source/_static/cover.png" alt="Awesome">
  <br>
  Design Engineering Staircase Robot
</h1>

<h4 align="center">
  <a href="http://de3-rob1-chess.rtfd.io">View the Project Documentation online</a>
  <br><br>
  <img width="80" src="http://readthedocs.org/projects/de3-rob1-chess/badge/?version=latest" alt="Documentation Status">
</h4>

<p align="center">
	<sub>Design Engineering, Imperial College London</sub>
</p>
<br>
<p align="center">
	<a href="https://vimeo.com/291377091" >
	<img width="600" src="vimeo.png" alt="Click to play"></a>
</h1>
<br>

## The Team

- Gabriele D'Amone 
- SeungHui Huh 
- YongXuam Li
- Huyang Liu 
- Fred Eric Macher 
- William Kwasi Pepera 
- Federico Tiersen 

## Contents
- Setting up Franka Emika Panda & Workstation
- Using MoveIt with the RViz & Gazebo Implementation
	- Introduction to MoveIt
	- Running the Pick and Place demo in RViz
	- Error when executing the RViz trajectory in Gazebo
	- Debugging
	- Exploring possible solutions / Giving up on MoveIt
- IK Solver
	- Picking the Brick
	- 

## Clone the repository

```bash
git clone http://github.com/nebbles/DE3-ROB1-CHESS
```

## Compiling the documentation offline

```bash
cd docs/
make html
open build/html/index.html
```

## Popular source code

* Source code for controlling Franka with Python (uses ROS)

```bash
svn export https://github.com/nebbles/DE3-ROB1-CHESS/trunk/franka/franka_control_ros.py
```

* Source code for converting between reference frames

```bash
svn export https://github.com/nebbles/DE3-ROB1-CHESS/trunk/tools/transform.py
```

![LICENSE](CC4.0-BY.jpg)
