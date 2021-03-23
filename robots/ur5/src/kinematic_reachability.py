#!/usr/bin/env python

from openravepy import *
import sys, os
import numpy as np
from math import sqrt, pi
env = Environment() # create the environment
#env.Load('data/lab1.env.xml') # load a scene
# env.Load('../../regular3dof/tridoftable.env.xml') # load a scene
env.Load('../xml/ur5-vertical.xml') # load a scene
# env.Load('../xml/ur5-with-objects.xml') # load a scene
env.SetViewer('qtcoin') # start the viewer
robot = env.GetRobots()[0] # get the first robot
print "Dof", robot.GetDOFValues()

manip = robot.GetActiveManipulator()
Tee = manip.GetEndEffectorTransform() # get end effector
robot.SetDOFValues([0.0,-1.57,1.57,-3.14,-1.57,0.0])


with env:
  ikmodel = databases.inversekinematics.InverseKinematicsModel(robot=robot,iktype=IkParameterization.Type.Translation3D)
  if not ikmodel.load():
		print "ikmodel is generating..."
		ikmodel.autogenerate()
		print "ikmodel is GENERATED..."
print "Load:", ikmodel.load()
print "Filename:", ikmodel.getfilename()
print "IKname:", ikmodel.getikname()

Tee = np.array([[0.00,  1.00,  0.00,  1.18], [1.00,  0.00,  0.00, -0.743], [-0.00,  0.00, -1.00,  1.011], [0.00,  0.00,  0.00,  1.00]])
print "Tee[0:3,3]:", Tee[0:3,3]
ikparam = IkParameterization(Tee[0:3,3],ikmodel.iktype) # build up the translation3d ik query
sol = manip.FindIKSolution(ikparam,IkFilterOptions.CheckEnvCollisions)

print "Ik solver:", sol

lower = np.concatenate((np.array([-0.01, 1.56, 1.56]), np.array([1., 1., 1.])*-3.14159265))
upper = np.concatenate((np.array([0.01, 1.58, 1.58]), np.array([1., 1., 1.])*3.14159265))
robot.SetDOFLimits(lower, upper)
print "DOF limits:", robot.GetDOFLimits()

# ikreachability = databases.kinematicreachability.ReachabilityModel(robot)
# filename = RaveFindDatabaseFile('/home/gizem/.openrave/robot.7edbf73fb4fc856e8294d93279d26ff2/reachability.12a06408e3f80af9e5cd98f6fe50e0ba.pp.')
ikreachability = databases.kinematicreachability.ReachabilityModel(robot=robot)
ikreachability.show()

    
raw_input('done')
sys.exit()

