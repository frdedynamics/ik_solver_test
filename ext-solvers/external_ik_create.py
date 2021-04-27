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


# Set environment and robot state
env = Environment()
env.Load(robot_path) # load a scene
env.SetViewer('qtcoin') # start the viewer
viewer = env.GetViewer()
# viewer.SetCamera([[0.,  0., -1., 2.],
# 					[1.,  0.,  0., 0.],
# 					[0., -1.,  0., .5],
# 					[0.,  0.,  0., 1.]])
viewer.SetCamera([[1.,  0., 0., 0.],
					[0.,  0.,  1., -1.5],
					[0., -1.,  0., .5],
					[0.,  0.,  0., 1.]])
robot = env.GetRobots()[0] # get the first robot
print "Dof", robot.GetDOFValues()
# RaveSetDebugLevel(DebugLevel.Debug)
		
iktype = IkParameterization.Type.Transform6D
	
ikmodel = databases.inversekinematics.InverseKinematicsModel(robot=robot,iktype=iktype)
if not ikmodel.load():
	print "New IK model is creating.."
	ikmodel.autogenerate()
	print "IK model created"
print "Load:", ikmodel.load()
print "Filename:", ikmodel.getfilename()
print "IKname:", ikmodel.getikname()
