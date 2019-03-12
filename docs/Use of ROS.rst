Need RQT Graph

In order to move the arm and the grippers, the arm publishers (8, 1 for each joint) and the gripper publishers (2, 1 for each gripper finger) are initialsed at the beginning of the main function of the source code. 

The topic subscribers from which relevant information is derived are the arm position (from the joint_states topic, calls the callback function which updates the "position" global variable) and the end effector position (from the gazebo/link_states topic, calls the ef_pos_get function which updates the "ef_pos" global variable).

From implementing this it is important to note the specific data type, i.e. JointState in the arm_pos variable initalisation, has to be imported from the data type category, in this case, "from sensor_msgs.msg import JointState". The category can be dervied from inputting into the terminal whilst the simulation is running: "rostopic type topic_name ". 

The topic in themselves often contain more information than required. For example, gazebo/link_states includes the states of all the links of the robot in gazebo, and so to get just the end_effector position, the pose attribute has to be indexed.

