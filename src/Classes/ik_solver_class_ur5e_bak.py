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

sys.path.append("/home/gizem/catkin_ws/src/ur5_with_hand_gazebo/src/Classes")
robot_dir = "/home/gizem/catkin_ws/src/ik_solver_test/robots/ur5e/xml/"
robot_name = "ur5e.xml"
robot_path = os.path.join(robot_dir, robot_name)
from DH_matrices import DHmatrices


_ROSTIME_START = 0
test_pub_msg = Vector3()
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
			self.iktype = IkParameterization.Type.Transform6D
		elif ikmodel==2:
			self.iktype = IkParameterization.Type.Translation3D
		else:
			sys.exit("IK type not known")
			
		self.ikmodel = databases.inversekinematics.InverseKinematicsModel(robot=self.robot,iktype=self.iktype)
		if not self.ikmodel.load():
			print "New IK model is creating.."
			self.ikmodel.autogenerate()
			print "IK model created"
		print "Load:", self.ikmodel.load()
		print "Filename:", self.ikmodel.getfilename()
		print "IKname:", self.ikmodel.getikname()

		sys.exit()

		# Set active manipulator bases
		self.basemanip = interfaces.BaseManipulation(self.robot)
		self.taskmanip = interfaces.TaskManipulation(self.robot)
		self.manip = self.robot.GetActiveManipulator()
		self.Tee_current = self.manip.GetEndEffectorTransform() # get end effector
		print "selected ee:", self.manip.GetEndEffector()

		print "Active Dofs:", self.robot.GetActiveDOF()
		self.robot.SetActiveDOFs([3, 4, 5])
		print "Active Dofs:", self.robot.GetActiveDOF()
		print self.robot.GetAffineDOF()

		lower = np.concatenate((np.array([-0.01, -(pi/2-0.01), pi/2-0.01]), np.array([1., 1., 1.])*-3.14159265))
		upper = np.concatenate((np.array([0.01, -(pi/2-0.01), pi/2+0.01]), np.array([1., 1., 1.])*3.14159265))
		self.robot.SetAffineRotationAxisLimits(lower,upper)
		print self.robot.GetAffineDOF()

		# Set joint limits
		null = [pi/2, -pi/2, 0.0, 0.0, 0.0, 0.0]
		home = [pi/2, -pi/2, 0.0, pi, -pi/2, 0.0]
		
		
		home2 = [pi/2, -pi/2., pi/2, pi, -pi/2, 0.0]
		self.robot.SetDOFValues(home2) 
		print "home:", self.manip.GetEndEffectorTransform() # get end effector
		dummy_input = raw_input("Next?")
		self.Twrist = self.robot.GetLink('wrist_1_link').GetTransform()
		print self.Twrist  ## don't update this. Initial is our pivot point. Otherwise it changes with wrist_1 rotation
		Tee_home = np.asarray(self.manip.GetEndEffectorTransform())
		print "Tee_home:", Tee_home

		test2 = [pi/2, -pi/2., pi/2, pi+pi/6, -pi/2, 0.0]
		self.robot.SetDOFValues(test2)
		print "test2:", self.manip.GetEndEffectorTransform() # get end effector
		dummy_input = raw_input("Next?")
		Tee2 = np.asarray(self.manip.GetEndEffectorTransform())
		print "Tee2:", Tee2

		test3 = [pi/2, -pi/2., pi/2, pi, -pi/2+pi/7, 0.0]
		self.robot.SetDOFValues(test3)
		print "test3:", self.manip.GetEndEffectorTransform() # get end effector
		dummy_input = raw_input("Next?")
		Tee3 = np.asarray(self.manip.GetEndEffectorTransform())
		print "Tee3:", Tee3

		lower = np.concatenate((np.array([-0.01, -(pi/2-0.01), pi/2-0.01]), np.array([1., 1., 1.])*-3.14159265))
		upper = np.concatenate((np.array([0.01, -(pi/2-0.01), pi/2+0.01]), np.array([1., 1., 1.])*3.14159265))
		# self.robot.SetDOFLimits(lower, upper)
		# print "DOF limits:", self.robot.GetDOFLimits()

		# # EE poses
		# Tee1 = np.array([[0.00,  -1.00,  0.00,  -0.0812], [1.00,  0.00,  0.00, 0.3922], [-0.00,  0.00, 1.00,  0.687], [0.00,  0.00,  0.00,  1.00]])
		# Tee2 = np.array([[0.00,  0.00,  -1.00,  0.496], [ 1.00,  0.00,  0.00, -0.743], [0.00,  -1.00, 0.00,  0.555], [ 0.00,  0.00,  0.00,  1.00]])
		# Tee3 = np.array([[1.00,  0.00, 0.00,  0.704], [0.00,  1.00, 0.00, -0.836], [0.00,  0.00,  1.00,  0.670], [0.00,  0.00,  0.00,  1.00]])
		# self.Tee_current = np.array([1.00,  0.00, 0.00,  0.00], [0.00,  1.00, 0.00,  0.00], [0.00,  0.00, 1.00,  0.00], [0.00,  0.00, 0.00,  1.00])
		# self.Tee_current = Tee1 ## for test only - constantly read by self.manip.GetEndEffectorTransform() # get end effector
		# self.Tee_goal = np.zeros((4,4), dtype=np.float32) # gonna be calculated by ik
		# self.Tee_goal = Tee1
	


		# IK parametrization init
		# self.ikparam = IkParameterization(self.Tee_current[0:3,3], self.ikmodel.iktype) # build up the translation3d ik query
		# self.sol = self.manip.FindIKSolution(self.ikparam, IkFilterOptions.CheckEnvCollisions)
		dummy_input = raw_input("Start Tee_home")
		self.ikparam = IkParameterization(Tee_home[0:3,3], self.ikmodel.iktype) # build up the translation3d ik query
		self.ikparam2 = IkParameterization(Tee_home, IkParameterization.Type.Transform6D) # build up the translation3d ik query
		dummy_input =raw_input("ikparam changed")
		self.sol = self.manip.FindIKSolutions(self.ikparam2, IkFilterOptions.CheckEnvCollisions)
		self.robot.SetDOFValues(self.sol, self.ikmodel.manip.GetArmIndices())
		dummy_input = raw_input("Next Tee2")

		self.ikparam = IkParameterization(Tee2[0:3,3], self.ikmodel.iktype) # build up the translation3d ik query
		self.sol = self.manip.FindIKSolution(self.ikparam, IkFilterOptions.CheckEnvCollisions)
		self.robot.SetDOFValues(self.sol, self.ikmodel.manip.GetArmIndices())
		dummy_input = raw_input("Next Tee3")

		self.ikparam = IkParameterization(Tee3[0:3,3], self.ikmodel.iktype) # build up the translation3d ik query
		self.sol = self.manip.FindIKSolution(self.ikparam, IkFilterOptions.CheckEnvCollisions)
		self.robot.SetDOFValues(self.sol, self.ikmodel.manip.GetArmIndices())
		dummy_input = raw_input("Done")
		sys.exit()


		# Init robot pose
		self.robot.SetDOFValues(self.sol, self.ikmodel.manip.GetArmIndices())

		Tee = self.manip.GetEndEffectorTransform()
		Tee_pose = DHmatrices.htm_to_pose(Tee)
		# print "Tee:", Tee
		dummy_input = raw_input("Done?")
		sys.exit()

		# Updated parameters
		self.joint_states = JointState()
		## TODO: parametrize such that robot.GetJointNames()
		self.joint_states.name = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
		self.joint_states.position = [0.0, 0.0, pi/2, 0.0, 0.0, 0.0]
		self.ee_goal = Vector3()
		self.test_joints = JointState()
		self.test_joints.position = [0.0,1.57,0.0,0.0,0.0,0.0]
		
		
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

		self.sub_Tee_pose = rospy.Subscriber('//Tee_mapper_goal_pose', Pose, self.sub_Tee_pose)
		self.sub_test_joint = rospy.Subscriber('/test_joints', JointState, self.sub_test_joint)
		# self.sub_selector = rospy.Subscriber('/selector', Int8, self.sub_selector)
		# self.log_start_time = rospy.get_time()
		_ROSTIME_START = rospy.get_time()
		print "ik_solver_node pub/sub initialized"


	def update(self):
		# tee_goal = self.Tee_goal
		# self.calculate_joint_angles2(tee_goal)

		# CALCULATE JOINT ANGLES AND PUBLISH BASED ON WHAT \human_robot_mapper SENT
		# self.Tee_goal = np.array([[0.00,  1.00,  0.00,  0.817], [1.00,  0.00,  0.00, -0.232], [0.00,  0.00, -1.00,  0.062], [0.00,  0.00,  0.00,  1.00]])
		self.calculate_joint_angles()
		self.joint_states.header.stamp = rospy.Time.now()
		self.pub.publish(self.joint_states)

		# CALCULATE TEE POSE IN OPENRV -- DEBUD PURPOSE
		self.Tee_current = self.manip.GetEndEffectorTransform()
		self.Tee_current_pose = DHmatrices.htm_to_pose(self.Tee_current)
		self.pub_calculated_tee.publish(self.Tee_current_pose)
		
		# print "self.test_joints.position", self.test_joints.position
		# self.robot.SetDOFValues(self.test_joints.position) 
		# print "Tee:", self.Tee_current
			

	def calculate_joint_angles(self):
		'''
		Given ee_goal, calculate joint angles. Do I need to pull ee_goal?
		@params ee_goal: type np.array(4x4) HTM
		'''
		global Tee_fail
		if (type(self.Tee_goal)==np.ndarray) and (self.Tee_goal.shape == (4,4)):
			comparison = self.Tee_goal == Tee_fail
			if not comparison.all():
				print "Tee_goal:", self.Tee_goal
				try:
					self.ikparam = IkParameterization(self.Tee_goal[0:3,3], self.ikmodel.iktype) # build up the translation3d ik query
					self.sols = self.manip.FindIKSolution(self.ikparam, IkFilterOptions.CheckEnvCollisions)
					self.robot.SetDOFValues(self.sols,self.ikmodel.manip.GetArmIndices())
					self.joint_states.position = self.robot.GetDOFValues()
					if self.sols is not None and len(self.sols) > 0: # if found, then break
						print len(self.sols)
						for sol in self.sols:
							print sol
					else:
						Tee_fail = self.Tee_goal
						raise openrave_exception("No solution")
				except openrave_exception, e:
					print e
				# print "Tee_goal:", self.Tee_goal
				print "joint positions:", self.joint_states.position
			else:
				print "The same invalid Tee"


		else:
			print "Unknown ee_type"
	
	def calculate_joint_angles2(self, tee_goal):
		'''
		Given ee_goal, calculate joint angles. Do I need to pull ee_goal?
		@params ee_goal: type np.array(4x4) HTM
		'''
		if (type(tee_goal)==np.ndarray) and (tee_goal.shape == (4,4)):
			self.ikparam = IkParameterization(tee_goal[0:3,3], self.ikmodel.iktype) # build up the translation3d ik query
			self.sol = self.manip.FindIKSolution(self.ikparam, IkFilterOptions.CheckEnvCollisions)
			self.robot.SetDOFValues(self.sol,self.ikmodel.manip.GetArmIndices())
			self.joint_states.position = self.robot.GetDOFValues()
			# print "Tee_goal:", tee_goal
			# print "joint positions:", self.joint_states.position

		else:
			print "Unknown ee_type"


	def sub_Tee_pose(self, msg):
		'''
		Subscribes Tee_pose {Pose()}, converts it to Tee {np.array()}
		'''
		Tee_goal_pose = msg
		print "Tee_goal_pose:", Tee_goal_pose
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
			
			
			
		


