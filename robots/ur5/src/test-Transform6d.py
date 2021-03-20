
from openravepy import *
import sys
import numpy as np
from math import sqrt, pi
env = Environment() # create the environment
#env.Load('data/lab1.env.xml') # load a scene
# env.Load('planar_3dof.xml') # load a scene
env.Load('../xml/ur5-with-objects.xml') # load a scene
env.SetViewer('qtcoin') # start the viewer
robot = env.GetRobots()[0] # get the first robot
print "Dof", robot.GetDOFValues()

manip = robot.GetActiveManipulator()
Tee = manip.GetEndEffectorTransform() # get end effector



# with env:
  # ikmodel = databases.inversekinematics.InverseKinematicsModel(robot=robot,iktype=IkParameterization.Type.Transform6D)
  # if not ikmodel.load():
    # ikmodel.autogenerate()
  # basemanip = interfaces.BaseManipulation(robot)
  # taskmanip = interfaces.TaskManipulation(robot)
  # robot.SetJointValues([-0.97],ikmodel.manip.GetGripperIndices())
  # robot.SetDOFValues([0.0,0.0,1.57,0.0,0.0,0.0])
  # Tstart = np.array([[0.0,  0.0007, -0.999999683,  0.496755774],
 # [ 1.00000000,  0.0,  0.0, -0.743859000],
 # [ 0.0, -0.999999683, 0.0,  0.554908046],
 # [ 0.00000000,  0.00000000,  0.00000000,  1.00000000]])
 
with env:
  ikmodel2 = databases.inversekinematics.InverseKinematicsModel(robot=robot,iktype=IkParameterization.Type.Translation3D)
  print "Load:", ikmodel2.load()
  print "Filename:", ikmodel2.getfilename()
  print "IKname:", ikmodel2.getikname()
  # print "Joint Names:", ikmodel2.getIndicesFromJointNames("")
  
  if not ikmodel2.load():
    ikmodel2.autogenerate()
  basemanip = interfaces.BaseManipulation(robot)
  taskmanip = interfaces.TaskManipulation(robot)
  robot.SetJointValues([-0.97],ikmodel2.manip.GetGripperIndices())
  robot.SetDOFValues([0.0,0.0,1.57,0.0,0.0,0.0])
  Tstart = np.array([[0.0,  0.0007, -0.999999683,  0.496755774],
 [ 1.00000000,  0.0,  0.0, -0.743859000],
 [ 0.0, -0.999999683, 0.0,  0.554908046],
 [ 0.00000000,  0.00000000,  0.00000000,  1.00000000]])

    
raw_input('Never here')
sys.exit()

