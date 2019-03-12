https://bitbucket.org/traclabs/trac_ik/src/HEAD/trac_ik_python/

The inverse kinematics solver used was trac_ik. It has a python wrapper which was used in order to include it in our solution. The solver uses quaternions as the inputs for the end effector orientation and Cartesian coordinates for the position. 

Initially, the function was not working as intended because our seed state was 0 rad for every joint and so the solver was returning the first solution as calculated from that arbitrary state. Subsequently, the function was changed such that it uses the current joint angles from the joint_states rostopic, written to the global variable "position".

from trac_ik_python.trac_ik import IK

...

def ik_solver(X, Y, Z, QX, QY, QZ, QW): # trac_ik inverse kinematics solver
    urdf_str = rospy.get_param('/robot_description')
    # print urdf_str
    ik_sol = IK("panda_link0","panda_link7",urdf_string=urdf_str)
    # print ik_sol.link_names

    global position
    seed_state = position[2:9]

    lower_bound, upper_bound = ik_sol.get_joint_limits()
    # print upper_bound
    # print lower_bound
    ik_sol.set_joint_limits(lower_bound, upper_bound)

    return ik_sol.get_ik(seed_state,
                X, Y, Z,  # X, Y, Z
                QX, QY, QZ, QW)  # QX, QY, QZ, QW
