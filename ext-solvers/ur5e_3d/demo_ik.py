import numpy as np
import ikfastpy
from math import pi
import sys

# Initialize kinematics for UR5 robot arm
# ur5_kin = ikfastpy.PyKinematics()
# n_joints = ur5_kin.getDOF()
# print n_joints

# joint_angles = [0.0,0.1,0.] # in radians

# # Test forward kinematics: get end effector pose from joint angles
# print("\nTesting forward kinematics:\n")
# print("Joint angles:")
# print(joint_angles)
# ee_pose = ur5_kin.forward(joint_angles)
# # ee_pose = np.asarray(ee_pose).reshape(3,4) # 3x4 rigid transformation matrix
# print("\nEnd effector pose:")
# print(ee_pose)
# print("\n-----------------------------")


# null_ee = np.array([[ 1., -0., 0., -0.817],
#                     [ 0., 0., -1., -0.234],
#                     [ 0., 1., -0., 0.063]])

# # Test inverse kinematics: get joint angles from end effector pose
# print("\nTesting inverse kinematics:\n")
# joint_configs = ur5_kin.inverse(ee_pose)
# n_solutions = int(len(joint_configs)/n_joints)
# print("%d solutions found:"%(n_solutions))
# joint_configs = np.asarray(joint_configs).reshape(n_solutions,n_joints)
# for joint_config in joint_configs:
#     print(joint_config)

# # Check cycle-consistency of forward and inverse kinematics
# assert(np.any([np.sum(np.abs(joint_config-np.asarray(joint_angles))) < 1e-4 for joint_config in joint_configs]))
# print("\nTest passed!")

sys.path.append("/home/gizem/catkin_ws/src/ik_solver_test/ext-solvers/ur5e_3d/Classes")
from ik_ur5e_translate_3d import IK_UR5ETRANS3D

IK = IK_UR5ETRANS3D()

joint_angles = np.array([0.0,0.1,0.]) # in radians
ee_calc = IK.calc_forward_kin(joint_angles.tolist())
print ee_calc

ee_coord = [0.1, 0.22650042, 0.00998334]
joints_calc = IK.calc_inverse_kin(ee_coord)
print joints_calc

current_joints = []