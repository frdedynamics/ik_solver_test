import numpy as np
import ikfastpy
from math import pi, sqrt
import sys
from geometry_msgs.msg import Pose, Point, Quaternion

# Initialize kinematics for UR5 robot arm
ur5_kin = ikfastpy.PyKinematics()
n_joints = ur5_kin.getDOF()
print n_joints

joint_angles = [0.0,0.1,0.] # in radians

# Test forward kinematics: get end effector pose from joint angles
print("\nTesting forward kinematics:\n")
print("Joint angles:")
print(joint_angles)
ee_pose = ur5_kin.forward(joint_angles)
# ee_pose = np.asarray(ee_pose).reshape(3,4) # 3x4 rigid transformation matrix
print("\nEnd effector pose:")
print(ee_pose)
print("\n-----------------------------")


null_ee = np.array([[ 1., -0., 0., -0.817],
                    [ 0., 0., -1., -0.234],
                    [ 0., 1., -0., 0.063]])

ee_pose_test = np.array([[ 1., 0., 0., 0.1000000015],
                        [ 0., 1., 0., 0.22650042],
                        [ 0., 1., 0., 0.00998334]])

ee_pose_test2 = np.array([[ 1., 0., 0., 0.1000000],
                        [ 0., 1., 0., 0.22650],
                        [ 0., 1., 0., 0.00998]])
                        

# Test inverse kinematics: get joint angles from end effector pose
print("\nTesting inverse kinematics:\n")
# joint_configs = ur5_kin.inverse(null_ee.reshape(-1).tolist())
joint_configs = ur5_kin.inverse(ee_pose_test.reshape(-1).tolist())
# joint_configs = ur5_kin.inverse(ee_pose)
n_solutions = int(len(joint_configs)/n_joints)
print("%d solutions found:"%(n_solutions))
joint_configs = np.asarray(joint_configs).reshape(n_solutions,n_joints)
for joint_config in joint_configs:
    print(joint_config)

# Check cycle-consistency of forward and inverse kinematics
assert(np.any([np.sum(np.abs(joint_config-np.asarray(joint_angles))) < 1e-4 for joint_config in joint_configs]))
print("\nTest passed!")

sys.exit()


## Accuracy test
print("\nTEST1:\n")
joint_configs_ee1 = ur5_kin.inverse(ee_pose_test.reshape(-1).tolist())
# joint_configs = ur5_kin.inverse(ee_pose)
n_solutions = int(len(joint_configs_ee1)/n_joints)
print("%d solutions found:"%(n_solutions))
joint_configs = np.asarray(joint_configs_ee1).reshape(n_solutions,n_joints)
for joint_config in joint_configs_ee1:
    print(joint_config)

# Check cycle-consistency of forward and inverse kinematics
assert(np.any([np.sum(np.abs(joint_config-np.asarray(joint_angles))) < 1e-4 for joint_config in joint_configs]))
print("\nTest passed!")

print("\nTEST2:\n")
joint_configs_ee2 = ur5_kin.inverse(ee_pose_test2.reshape(-1).tolist())
n_solutions = int(len(joint_configs_ee2)/n_joints)
print("%d solutions found:"%(n_solutions))
joint_configs = np.asarray(joint_configs_ee2).reshape(n_solutions,n_joints)
for joint_config in joint_configs_ee2:
    print(joint_config)



#########################  With class #############################
# sys.path.append("/home/gizem/catkin_ws/src/ik_solver_test/ext-solvers/ur5e_3d/Classes")
# from ik_ur5e_translate_3d import IK_UR5ETRANS3D

# IK = IK_UR5ETRANS3D()

# sys.path.append("/home/gizem/catkin_ws/src/ur5_with_hand_gazebo/src/Classes")
# from DH_matrices import DHmatrices

# joint_angles = [0.03, 0.0, 0.0]
# ee = IK.calc_forward_kin(joint_angles)

# def mag(ee):
#     r = sqrt((ee[0]**2) + (ee[1]**2) + (ee[2]**2))
#     return r
# print ee, mag(ee)
# sys.exit()

# ee_coord = [0.1, 0.22650042, 0.00998334]
# ee_coord = [0.001570, 0.247784,  -0.000309]
# # hand_pose = Pose(Point(ee_coord[0], ee_coord[1], ee_coord[2]), Quaternion(0., 0., 0., 1.))
# # wrist_pose = Pose(Point(0., 0., 0.), Quaternion(0., 0., 0., 1.))
# # param_x = DHmatrices.ee_goal_calculate(hand_pose, wrist_pose, param='x')
# # sys.exit(param_x)
# joints_calc = IK.calc_inverse_kin(ee_coord)
# print joints_calc

# current_joints = np.array([0.0, 0.1, 0.0]).tolist()
# print IK.choose_closest_soln(current_joints)