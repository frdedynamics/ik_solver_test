
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
  # print "Load:", ikmodel.load()
  # print "Filename:", ikmodel.getfilename()
  # print "IKname:", ikmodel.getikname()
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
  # handle = manip.GetIkSolver()
  # print "handle:", handle
  # success = manip.FindIKSolution(Tstart, IkFilterOptions.CheckEnvCollisions)
  # print "Ik solver:", success # Ik solver: [ 2.26854624e-08  1.35385410e-01  1.48409766e+00  7.36569097e-01 -4.95077482e-04 -7.85605442e-01]

 
with env:
  # ikmodel2 = databases.inversekinematics.InverseKinematicsModel(robot=robot,iktype=IkParameterization.Type.Translation3D)
  ikmodel2 = databases.inversekinematics.InverseKinematicsModel(robot=robot,iktype=IkParameterization.Type.Translation3D)
  print "Load:", ikmodel2.load()
  print "Filename:", ikmodel2.getfilename()
  print "IKname:", ikmodel2.getikname()
  freeparam = manip.GetIkSolver().GetNumFreeParameters()
  print "IK free param:", freeparam
  # print "Joint Names:", ikmodel2.getIndicesFromJointNames("")
  
  if not ikmodel2.load():
    ikmodel2.autogenerate()
  basemanip = interfaces.BaseManipulation(robot)
  taskmanip = interfaces.TaskManipulation(robot)
  robot.SetJointValues([-0.97],ikmodel2.manip.GetGripperIndices())
  robot.SetDOFValues([0.0,0.0,1.57,0.0,0.0,0.0])
  Tstart = np.array([[0.0,  0.0007, -0.999999683,  0.496755774], [ 1.00000000,  0.0,  0.0, -0.743859000],[ 0.0, -0.999999683, 0.0,  0.554908046], [ 0.00000000,  0.00000000,  0.00000000,  1.00000000]])
  ikparam = IkParameterization(Tee[0:3,3],ikmodel2.iktype) # build up the translation3d ik query
# iksolverbase = InterfaceBase.IkSolverBase.Init(robot) I don't know how to use IkSolverBase
handle = manip.GetIkSolver()
success = manip.FindIKSolution(ikparam,IkFilterOptions.CheckEnvCollisions)
print "Ik solver:", success # Ik solver: [ 5.55111512e-16 -3.96993549e-12  1.57000000e+00  1.11022302e-16  0.00000000e+00  0.00000000e+00] with manip.GetIkParameterization(IkParameterization.Type.Translation3D)
print "Ik solver:", success # Ik solver: [ [-1.11022302e-16  8.99614938e-10  1.57000000e+00  1.11022302e-16  0.00000000e+00  0.00000000e+00]] with ikparam



# print "testing..."
# successrate, wrongrate = ikmodel2.testik(str(100))
# print "iktest:", successrate, wrongrate
 



    
raw_input('Never here')
sys.exit()

