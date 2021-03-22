#!/usr/bin/env python

"""
Subscribes T_goal, publishes joint angles

"""

# imports
# import Data.data_logger_module as data_logger


import rospy
import numpy as np
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Vector3

_ROSTIME_START = 0


class IKSolver:
	def __init__(self, ikmodel=2, initnode=False, rate=100):
		''' Initializes the openrave environment, robot state and IK model
		@params ikmodel: 1->Transform6D 2->Translation3D
		'''

		# Set environment and robot state
		self.env = Environment()
		self.env.Load('../xml/ur5-with-objects.xml') # load a scene
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
			
		self.ikmodel = databases.inversekinematics.InverseKinematicsModel(robot=robot,iktype=self.iktype)
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
		self.manip = robot.GetActiveManipulator()
		self.Tee = manip.GetEndEffectorTransform() # get end effector

		# Set joint limits
		lower = np.concatenate((np.array([-0.01, -0.01, pi/2-0.01]), np.array([1., 1., 1.])*-3.14159265))
		upper = np.concatenate((np.array([0.01, 0.01, pi/2+0.01]), np.array([1., 1., 1.])*3.14159265))
		self.robot.SetDOFLimits(lower, upper)
		print "DOF limits:", robot.GetDOFLimits()

		# EE poses
		Tee1 = np.array([[0.00,  1.00,  0.00,  1.18], [1.00,  0.00,  0.00, -0.743], [-0.00,  0.00, -1.00,  1.011], [0.00,  0.00,  0.00,  1.00]])
		Tee2 = np.array([[0.00,  0.00,  -1.00,  0.496], [ 1.00,  0.00,  0.00, -0.743], [0.00,  -1.00, 0.00,  0.555], [ 0.00,  0.00,  0.00,  1.00]])
		Tee3 = np.array([[1.00,  0.00, 0.00,  0.704], [0.00,  1.00, 0.00, -0.836], [0.00,  0.00,  1.00,  0.670], [0.00,  0.00,  0.00,  1.00]])
		# self.Tee = np.array([1.00,  0.00, 0.00,  0.00], [0.00,  1.00, 0.00,  0.00], [0.00,  0.00, 1.00,  0.00], [0.00,  0.00, 0.00,  1.00])
		self.Tee = Tee1

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
		
		self.r = rospy.Rate(rate)
		if initnode == True:
			rospy.init_node("ik_solver_node")
			print "ik_solver_node initialized"
				
			    
    def init_subscribers_and_publishers(self):
			self.pub = rospy.Publisher('/joint_states', JointState, queue_size=1)
			self.sub_ee_goal = rospy.Subscriber('/ee_goal', Vector3, self.sub_ee_goal)
			self.sub_imu_s = rospy.Subscriber('/sensor_l_shoulder', Imu, self.cb_imu_shoulder)
			self.sub_imu_e = rospy.Subscriber('/sensor_l_elbow', Imu, self.cb_imu_elbow)
			self.sub_imu_w = rospy.Subscriber('/sensor_l_wrist', Imu, self.cb_imu_wrist)
			# self.log_start_time = rospy.get_time()
			_ROSTIME_START = rospy.get_time()
			print "ik_solver_node pub/sub initialized"


    def update(self):
			self.calculate_joint_angles(self.Tee)
			self.joint_states.header.stamp = rospy.Time.now()
			self.pub.publish(self.joint_states)
			

		def calculate_joint_angles(self, Tee):
			'''
			Given ee_goal, calculate joint angles. Do I need to pull ee_goal?
			@params ee_goal: type np.array(4x4) HTM
			'''
			if (type(Tee)==np.ndarray) and (Tee.shape == (4,4)):
				# self.Tee = Tee
				self.ikparam = IkParameterization(self.Tee[0:3,3], self.ikmodel.iktype) # build up the translation3d ik query
				self.sol = self.manip.FindIKSolution(self.ikparam, IkFilterOptions.CheckEnvCollisions)
				self.robot.SetDOFValues(self.sol,self.ikmodel.manip.GetArmIndices())
				self.joint_states.position = self.robot.GetDOFValues()

			else:
				print "Unknown ee_type"


    def sub_ee_goal(self, msg):
        self.ee_goal = msg
        while self.calibration_flag < _CALIBRATION_TH:
            self.q_chest_init = kinematic.q_invert(self.chest_measurement.orientation)
            # print "calibrating chest"
        self.q_chest = kinematic.q_multiply(self.q_chest_init, self.chest_measurement.orientation)
        self.acc_chest = self.chest_measurement.linear_acceleration
        self.gyro_chest = self.chest_measurement.angular_velocity
        self.chest_angles = q2e(kinematic.q_tf_convert(self.q_chest), axes='sxyz')

    def cb_imu_shoulder(self, msg):
        self.shoulder_measurement = msg
        while self.calibration_flag < _CALIBRATION_TH:
            self.q_shoulder_init = kinematic.q_invert(self.shoulder_measurement.orientation)
            # print "calibrating shoulder"
        self.q_shoulder = kinematic.q_multiply(self.q_shoulder_init, self.shoulder_measurement.orientation)
        # print "q_measured: {0} \n q_init:{1}".format(self.shoulder_measurement.orientation, self.q_shoulder_init)
        self.shoulder_angles = q2e(kinematic.q_tf_convert(self.q_shoulder), axes='sxyz')
        self.acc_shoulder = self.shoulder_measurement.linear_acceleration
        self.gyro_shoulder = self.shoulder_measurement.angular_velocity
        # q_shoulder_sensorframe = kinematic.q_multiply(kinematic.q_invert(self.q_chest), self.q_shoulder)
        # self.shoulder_angles = q2e(kinematic.q_tf_convert(q_shoulder_sensorframe), axes='sxyz')
        # Update joint angles
        self.human_joint_imu.position[0] = self.shoulder_angles[0]  # pitch
        self.human_joint_imu.position[1] = self.shoulder_angles[1]  # yaw
        self.human_joint_imu.position[2] = self.shoulder_angles[2]  # roll

    def cb_imu_elbow(self, msg):
        self.elbow_measurement = msg
        while self.calibration_flag < _CALIBRATION_TH:
            self.q_elbow_init = kinematic.q_invert(self.elbow_measurement.orientation)
            # print "calibrating elbow"
        self.q_elbow = kinematic.q_multiply(self.q_elbow_init, self.elbow_measurement.orientation)
        q_elbow_sensorframe = kinematic.q_multiply(kinematic.q_invert(self.q_shoulder), self.q_elbow)
        self.elbow_angles = q2e(kinematic.q_tf_convert(q_elbow_sensorframe), axes='sxyz')
        self.acc_elbow = self.elbow_measurement.linear_acceleration
        self.gyro_elbow = self.elbow_measurement.angular_velocity
        # Update joint angles
        self.human_joint_imu.position[3] = self.elbow_angles[0]  # pitch
        self.human_joint_imu.position[4] = self.elbow_angles[1]  # yaw
        self.human_joint_imu.position[5] = self.elbow_angles[2]  # roll

    def cb_imu_wrist(self, msg):
        self.wrist_measurement = msg
        while self.calibration_flag < _CALIBRATION_TH:
            self.q_wrist_init = kinematic.q_invert(self.wrist_measurement.orientation)
            # print "calibrating wrist"
        self.q_wrist = kinematic.q_multiply(self.q_wrist_init, self.wrist_measurement.orientation)
        q_wrist_sensorframe = kinematic.q_multiply(kinematic.q_invert(self.q_elbow), self.q_wrist)
        self.wrist_angles = q2e(kinematic.q_tf_convert(q_wrist_sensorframe), axes='sxyz')
        self.acc_wrist = self.wrist_measurement.linear_acceleration
        self.gyro_wrist = self.wrist_measurement.angular_velocity
        # Update joint angles
        self.human_joint_imu.position[6] = self.wrist_angles[0]  # pitch
        self.human_joint_imu.position[7] = self.wrist_angles[1]  # yaw
        self.human_joint_imu.position[8] = self.wrist_angles[2]  # roll
