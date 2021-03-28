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

from openravepy import *

sys.path.append("/home/gizem/catkin_ws/src/ur5_with_hand_gazebo/src/Classes")
robot_dir = "/home/gizem/catkin_ws/src/ik_solver_test/robots/ur5/xml/"
robot_name = "ur5-with-objects.xml"
robot_path = os.path.join(robot_dir, robot_name)
from DH_matrices import DHmatrices


_ROSTIME_START = 0


class IKSolver:
	def __init__(self, ikmodel=2, START_NODE=False, rate=100):
		''' Initializes the openrave environment, robot state and IK model
		@params ikmodel: 1->Transform6D 2->Translation3D
		'''

		# Set environment and robot state
		self.env = Environment()
		self.env.Load(robot_path) # load a scene
		self.env.SetViewer('qtcoin') # start the viewer
		self.robot = self.env.GetRobots()[0] # get the first robot
		print "Dof", self.robot.GetDOFValues()

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

		# Set active manipulator bases
		self.basemanip = interfaces.BaseManipulation(self.robot)
		self.taskmanip = interfaces.TaskManipulation(self.robot)
		self.manip = self.robot.GetActiveManipulator()
		self.Tee = self.manip.GetEndEffectorTransform() # get end effector

		# Set joint limits
		self.robot.SetDOFValues([0.0,-1.57,1.57,0.0,0.0,0.0]) ## you may need to check this values.
		lower = np.concatenate((np.array([-0.01, -(pi/2-0.01), pi/2-0.01]), np.array([1., 1., 1.])*-3.14159265))
		upper = np.concatenate((np.array([0.01, -(pi/2-0.01), pi/2+0.01]), np.array([1., 1., 1.])*3.14159265))
		self.robot.SetDOFLimits(lower, upper)
		print "DOF limits:", self.robot.GetDOFLimits()

		# EE poses
		Tee1 = np.array([[0.00,  1.00,  0.00,  1.18], [1.00,  0.00,  0.00, -0.743], [-0.00,  0.00, -1.00,  1.011], [0.00,  0.00,  0.00,  1.00]])
		Tee2 = np.array([[0.00,  0.00,  -1.00,  0.496], [ 1.00,  0.00,  0.00, -0.743], [0.00,  -1.00, 0.00,  0.555], [ 0.00,  0.00,  0.00,  1.00]])
		Tee3 = np.array([[1.00,  0.00, 0.00,  0.704], [0.00,  1.00, 0.00, -0.836], [0.00,  0.00,  1.00,  0.670], [0.00,  0.00,  0.00,  1.00]])
		# self.Tee = np.array([1.00,  0.00, 0.00,  0.00], [0.00,  1.00, 0.00,  0.00], [0.00,  0.00, 1.00,  0.00], [0.00,  0.00, 0.00,  1.00])
		self.Tee = Tee1 ## for test only

		# IK parametrization init
		self.ikparam = IkParameterization(self.Tee[0:3,3], self.ikmodel.iktype) # build up the translation3d ik query
		self.sol = self.manip.FindIKSolution(self.ikparam, IkFilterOptions.CheckEnvCollisions)

		# Init robot pose
		self.robot.SetDOFValues(self.sol, self.ikmodel.manip.GetArmIndices())

		# Updated parameters
		self.joint_states = JointState()
		## TODO: parametrize such that robot.GetJointNames()
		self.joint_states.name = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
		self.joint_states.position = [0.0, 0.0, pi/2, 0.0, 0.0, 0.0]
		self.ee_goal = Vector3()
		
		
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
		self.pub = rospy.Publisher('/joint_states', JointState, queue_size=1) # /ur5_joint_position_cmd
		# self.sub_hand_pose = rospy.Subscriber('/hand_pose', Pose, self.sub_hand_pose)
		# self.sub_wrist_pose = rospy.Subscriber('/wrist_pose', Pose, self.sub_hand_pose)
		self.sub_Tee_pose = rospy.Subscriber('/Tee_pose', Pose, self.sub_Tee_pose)
		# self.log_start_time = rospy.get_time()
		_ROSTIME_START = rospy.get_time()
		print "ik_solver_node pub/sub initialized"


	def update(self):
		self.calculate_joint_angles()
		self.joint_states.header.stamp = rospy.Time.now()
		self.pub.publish(self.joint_states)
			

	def calculate_joint_angles(self):
		'''
		Given ee_goal, calculate joint angles. Do I need to pull ee_goal?
		@params ee_goal: type np.array(4x4) HTM
		'''
		if (type(self.Tee)==np.ndarray) and (self.Tee.shape == (4,4)):
			# self.Tee = Tee
			self.ikparam = IkParameterization(self.Tee[0:3,3], self.ikmodel.iktype) # build up the translation3d ik query
			self.sol = self.manip.FindIKSolution(self.ikparam, IkFilterOptions.CheckEnvCollisions)
			self.robot.SetDOFValues(self.sol,self.ikmodel.manip.GetArmIndices())
			self.joint_states.position = self.robot.GetDOFValues()

		else:
			print "Unknown ee_type"


	def sub_Tee_pose(self, msg):
		'''
		Subscribes Tee_pose {Pose()}, converts it to Tee {np.array()}
		'''
		Tee_pose = msg
		self.Tee = DHmatrices.pose_to_htm(Tee_pose)
			
		


