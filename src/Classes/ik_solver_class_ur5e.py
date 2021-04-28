#!/usr/bin/env python

"""
Subscribes T_goal {Pose()}, publishes joint angles {JointState()}

"""

# imports
# import Data.data_logger_module as data_logger

import sys, os
import rospy
from math import sqrt, pi
import numpy as np
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Pose
from std_msgs.msg import Int8

from openravepy import *

robot_dir = "/home/gizem/catkin_ws/src/ik_solver_test/robots/ur5e/xml/"
robot_name = "ur5e.xml"
robot_path = os.path.join(robot_dir, robot_name)


sys.path.append("/home/gizem/catkin_ws/src/ur5_with_hand_gazebo/src/Classes")
from DH_matrices import DHmatrices

# sys.path.append("/home/gizem/catkin_ws/src/ik_solver_test/ext-solvers/ur5e_3d/Classes")
# from ik_ur5e_translate_3d import IK_UR5ETRANS3D
sys.path.append("/home/gizem/catkin_ws/src/ik_solver_test/ext-solvers/ur5e_6d/Classes")
from ik_ur5e_transform_6d import IK_UR5ETRANSFORM6D

Tee_fail = np.zeros((4,4))


class IKSolver:
	def __init__(self, ikmodel=1, START_NODE=False, rate=100):
		''' Initializes the openrave environment, robot state and IK model
		@params ikmodel: 1->Transform6D 2->Translation3D
		'''

		# Set environment and robot state
		self.env = Environment()
		self.env.Load(robot_path) # load a scene
		self.env.SetViewer('qtcoin') # start the viewer
		viewer = self.env.GetViewer()
		# viewer.SetCamera([[0.,  0., -1., 2.],
		# 					[1.,  0.,  0., 0.],
		# 					[0., -1.,  0., .5],
		# 					[0.,  0.,  0., 1.]])
		viewer.SetCamera([[1.,  0., 0., 0.],
							[0.,  0.,  1., -1.5],
							[0., -1.,  0., .5],
							[0.,  0.,  0., 1.]])
		self.robot = self.env.GetRobots()[0] # get the first robot
		print "Dof", self.robot.GetDOFValues()
		# RaveSetDebugLevel(DebugLevel.Debug)
				

		# Set IK model
		if ikmodel==1:
			self.iksolver = IK_UR5ETRANSFORM6D()
		elif ikmodel==2:
			self.iksolver = IK_UR5ETRANS3D()
		else:
			sys.exit("IK type not known")


		# Set active manipulator bases
		self.basemanip = interfaces.BaseManipulation(self.robot)
		self.taskmanip = interfaces.TaskManipulation(self.robot)
		self.manip = self.robot.GetActiveManipulator()
		self.Tee_current = self.manip.GetEndEffectorTransform() # get end effector
		print "selected ee:", self.manip.GetEndEffector()


		# Initial poses
		home = [pi/2, -pi/2., pi/2, pi, -pi/2, 0.0]
		# self.robot.SetDOFValues(home) 
		# dummy_input = raw_input("Next?")
		# home = [pi/2, -pi/2., pi/2, 0.0,0.1,0.]
		# self.robot.SetDOFValues(home) 
		# dummy_input = raw_input("Next?")
		# home = [pi/2, -pi/2., pi/2, 0.0,0.1,0.3]
		# self.robot.SetDOFValues(home) 
		# dummy_input = raw_input("Next?")
		# home = [pi/2, -pi/2., pi/2, 1.0,-0.2,0.]
		# self.robot.SetDOFValues(home) 
		# dummy_input = raw_input("Next?")
		# home = [pi/2, -pi/2., pi/2, -1.4,-0.7,0.]
		# self.robot.SetDOFValues(home) 
		# dummy_input = raw_input("Next?")
		# home = [pi/2, -pi/2., pi/2, -1.4,-1.9,0.]
		# self.robot.SetDOFValues(home) 
		# dummy_input = raw_input("Next?")
		# home = [pi/2, -pi/2., pi/2, -1.4,-2.7,0.]
		# self.robot.SetDOFValues(home) 
		# dummy_input = raw_input("Next?")
		# sys.exit(0)
		self.robot.SetDOFValues(home) 
		# dummy_input = raw_input("Next?")
		Tee_home = np.asarray(self.manip.GetEndEffectorTransform())
		print "Tee_home:", self.manip.GetEndEffectorTransform() # get end effector

		self.Twrist = self.robot.GetLink('wrist_1_link').GetTransform()
		print self.Twrist  ## don't update this. Initial is our pivot point. Otherwise it changes with wrist_1 rotation


		# Updated parameters
		self.joint_states = JointState()
		## TODO: parametrize such that robot.GetJointNames()
		self.joint_states.name = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
		self.joint_states.position = home
		
		# self.test_joints = JointState()
		# self.test_joints.position = [0.0,1.57,0.0,0.0,0.0,0.0]

		self.Tee_goal = np.eye(4,4)
		self.Tee_current = Pose() ## To publish /Tee_calculated for DEBUG
		
		if START_NODE == True:
			rospy.init_node("ik_solver_node")
			self.r = rospy.Rate(rate)
			print "ik_solver_node initialized"


	def start_node(self, rate):
		'''
		NOTE: only one node should be calling this function.
		'''
		rospy.init_node('ik_solver_node', anonymous=False)
		print "ik_solver_node initialized"
		# self.r = rospy.Rate(rate)
				
			    
	def init_subscribers_and_publishers(self):
		self.pub = rospy.Publisher('/joint_states_openrave', JointState, queue_size=1) # calculated joints for ur5
		self.pub_calculated_tee = rospy.Publisher('/Tee_calculated', Pose, queue_size=1) # For debug purposes
		# self.pub_test = rospy.Publisher('/test_msg', Vector3, queue_size=1) #

		self.sub_Tee_pose = rospy.Subscriber('/Tee_goal_pose', Pose, self.sub_Tee_pose)
		self.sub_test_joint = rospy.Subscriber('/test_joints', JointState, self.sub_test_joint)
		# self.sub_selector = rospy.Subscriber('/selector', Int8, self.sub_selector)
		# self.log_start_time = rospy.get_time()
		_ROSTIME_START = rospy.get_time()
		print "ik_solver_node pub/sub initialized"

		ee_pose_test2 = np.array([[0.35469, -0.51845, -0.77807, 0.520], [0.82533, 0.56464, 0.0, 0.081], [0.43933, -0.64217, 0.62817, -0.332], [0.00,  0.00,  0.00,  1.00]])
		print "Goal:", DHmatrices.htm_to_pose(ee_pose_test2)


	def update(self):
		# tee_goal = self.Tee_goal
		# self.calculate_joint_angles2(tee_goal)

		# CALCULATE JOINT ANGLES AND PUBLISH BASED ON WHAT \human_robot_mapper SENT
		# self.Tee_goal = np.array([[0.00,  1.00,  0.00,  0.817], [1.00,  0.00,  0.00, -0.232], [0.00,  0.00, -1.00,  0.062], [0.00,  0.00,  0.00,  1.00]])
		self.calculate_joint_angles_fullarm()
		self.joint_states.header.stamp = rospy.Time.now()
		self.pub.publish(self.joint_states)

		# CALCULATE TEE POSE IN OPENRV -- DEBUD PURPOSE
		self.Tee_current = self.manip.GetEndEffectorTransform()
		self.Tee_current_pose = DHmatrices.htm_to_pose(self.Tee_current)
		self.pub_calculated_tee.publish(self.Tee_current_pose)
		
		# print "self.test_joints.position", self.test_joints.position
		# self.robot.SetDOFValues(self.test_joints.position) 
		# print "Tee:", self.Tee_current
			

	def calculate_joint_angles_wrist(self):
		'''
		Given ee_goal, calculate joint angles. Do I need to pull ee_goal?
		@params ee_goal: type np.array(4x4) HTM
		'''
		global Tee_fail
		if (type(self.Tee_goal)==np.ndarray) and (self.Tee_goal.shape == (4,4)):
			comparison = self.Tee_goal == Tee_fail
			if not comparison.all():
				print "Tee_goal:", self.Tee_goal[0:3,3]
				tpose = [0.24770613, 0.00644665, -0.00000013]
				tpose = [0.1, 0.22650042, 0.00998334]

				try: 
					ur_wrist_joints_all = self.iksolver.calc_inverse_kin(self.Tee_goal.reshape(-1).tolist())
					# ur_wrist_joints_all = self.iksolver.calc_inverse_kin(tpose)
					if self.iksolver.n_solutions > 0:
						ur_wrist_joints = self.iksolver.choose_closest_soln(self.joint_states.position[3:])
						print "calculated joints:", ur_wrist_joints
						self.joint_states.position[3] = ur_wrist_joints[0]
						self.joint_states.position[4] = ur_wrist_joints[1]
						self.joint_states.position[5] = ur_wrist_joints[2]
					else:
						Tee_fail = self.Tee_goal
						# raise openrave_exception("No solution")
				except openrave_exception, e:
					print e
				# print "Tee_goal:", self.Tee_goal
				# print "joint positions:", self.joint_states.position
			else:
				# print "The same invalid Tee"
				pass


		else:
			print "Unknown ee_type"


	def calculate_joint_angles_fullarm(self):
		'''
		Given ee_goal, calculate joint angles. Do I need to pull ee_goal?
		@params ee_goal: type np.array(4x4) HTM
		'''
		global Tee_fail
		if (type(self.Tee_goal)==np.ndarray) and (self.Tee_goal.shape == (4,4)):
			comparison = self.Tee_goal == Tee_fail
			if not comparison.all():
				print "Tee_goal:", self.Tee_goal[0:3,3]

				# ee = self.iksolver.calc_forward_kin([0.1,-0.75,0.2,1.5,-0.6,0.])
				ee_pose_test = np.array([[ 0.35469353199005127, -0.5184540748596191, -0.7780731916427612, 0.5204400420188904],
                    [ 0.8253356218338013, 0.5646424889564514, 0.0, 0.0812000036239624],
                    [ 0.4393332004547119, -0.6421715021133423, 0.6281736493110657, -0.33196911215782166]])

				try: 
					# ur_wrist_joints_all = self.iksolver.calc_inverse_kin(self.Tee_goal[0:3,3].tolist())
					ur_joints_all = self.iksolver.calc_inverse_kin(ee_pose_test)
					if self.iksolver.n_solutions > 0:
						ur_selected_joints = self.iksolver.choose_closest_soln(self.joint_states.position)
						print "calculated joints:", ur_selected_joints
						self.joint_states.position[0] = ur_selected_joints[0]
						self.joint_states.position[1] = ur_selected_joints[1]
						self.joint_states.position[2] = ur_selected_joints[2]
						self.joint_states.position[3] = ur_selected_joints[3]
						self.joint_states.position[4] = ur_selected_joints[4]
						self.joint_states.position[5] = ur_selected_joints[5]
					else:
						Tee_fail = self.Tee_goal
						raise openrave_exception("No solution")
				except openrave_exception, e:
					print e
				# print "Tee_goal:", self.Tee_goal
				# print "joint positions:", self.joint_states.position
			else:
				print "The same invalid Tee"
				pass
		else:
			print "Unknown ee_type"
	


	def sub_Tee_pose(self, msg):
		'''
		Subscribes Tee_pose {Pose()}, converts it to Tee {np.array()}
		'''
		Tee_goal_pose = msg
		# print "Tee_goal_pose:", Tee_goal_pose
		mag = sqrt((Tee_goal_pose.position.x**2) + (Tee_goal_pose.position.y**2) + (Tee_goal_pose.position.z**2))
		self.Tee_goal = DHmatrices.pose_to_htm(Tee_goal_pose)
		
		
	def sub_test_joint(self, msg):
		'''
		Subscribes Tee_pose {Pose()}, converts it to Tee {np.array()}
		'''
		self.test_joints.position = list(msg.position)
		
	def sub_selector(self, msg):
		'''
		This is only for test purpose 
		'''
		
		selector = msg.data
		print "Selection detected", selector
		if selector == 1:
			self.Tee_goal = np.array([[0.00,  1.00,  0.00,  0.817], [1.00,  0.00,  0.00, -0.232], [0.00,  0.00, -1.00,  0.062], [0.00,  0.00,  0.00,  1.00]])
		elif selector == 2:
			self.Tee_goal = np.array([[0.0, 1.0, -0.01, 0.462], [1.0, 0.0, 0.0, -0.743], [0.0, -0.01, -1.0, 1.734], [0.0, 0.0, 0.0, 1.0]])
		elif selector == 3:
			self.Tee_goal = np.array([[0.0, 1.0, -0.01, 0.4352], [1.0, 0.0, 0.0, -0.324243], [0.0, -0.01, -1.0, 2.234], [0.0, 0.0, 0.0, 1.0]])
		else:
			print "non registered selection"
			
			
			
		


