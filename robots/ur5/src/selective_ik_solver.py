
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
  ikmodel2 = databases.inversekinematics.InverseKinematicsModel(robot=robot,iktype=IkParameterization.Type.Translation3D)
  print "Load:", ikmodel2.load()
  print "Filename:", ikmodel2.getfilename()
  print "IKname:", ikmodel2.getikname()
  freeparam = manip.GetIkSolver().GetFreeParameters() # from: http://openrave.org/docs/latest_stable/coreapihtml/classOpenRAVE_1_1IkSolverBase.html
  print "IK free param:", freeparam
  weights = np.array([10.0, 10.0, 10.0, 0.1, 0.1, 0.1])
  # weights = np.ones(robot.GetDOF())
  robot.SetDOFWeights(weights)
  print "DOF weights:", robot.GetDOFWeights()
  
# robot.SetDOFValues([0.0,0.0,0.0,0.0,0.0,0.0])
# print "Tee-1:", manip.GetEndEffectorTransform() # get end effector
# robot.SetDOFValues([0.0,0.0,1.57,0.0,0.0,0.0])
# print "Tee-2:", manip.GetEndEffectorTransform() # get end effector

# Tee-1: [[-4.89630558e-12  1.00000000e+00  9.79261117e-12  1.18430000e+00]
 # [ 1.00000000e+00  4.89630558e-12  0.00000000e+00 -7.43859000e-01]
 # [-4.79473950e-23  9.79261117e-12 -1.00000000e+00  1.01160000e+00]
 # [ 0.00000000e+00  0.00000000e+00  0.00000000e+00  1.00000000e+00]]
# Tee-2: [[-3.83026943e-15  7.96326721e-04 -9.99999683e-01  4.96755774e-01]
 # [ 1.00000000e+00  4.89647212e-12  1.11022302e-16 -7.43859000e-01]
 # [ 4.89641661e-12 -9.99999683e-01 -7.96326721e-04  5.54908046e-01]
 # [ 0.00000000e+00  0.00000000e+00  0.00000000e+00  1.00000000e+00]]



  
if not ikmodel2.load():
	ikmodel2.autogenerate()
basemanip = interfaces.BaseManipulation(robot)
taskmanip = interfaces.TaskManipulation(robot)
# robot.SetJointValues([-0.97],ikmodel2.manip.GetGripperIndices())
robot.SetDOFValues([0.0,0.0,0.0,0.0,0.0,0.0])

# Tstart = np.array([[0.0,  0.0007, -0.999999683,  0.496755774], [ 1.00000000,  0.0,  0.0, -0.743859000],[ 0.0, -0.999999683, 0.0,  0.554908046], [ 0.00000000,  0.00000000,  0.00000000,  1.00000000]])
Tee1 = np.array([[0.00,  1.00,  0.00,  1.18], [1.00,  0.00,  0.00, -0.743], [-0.00,  0.00, -1.00,  1.011], [0.00,  0.00,  0.00,  1.00]])
Tee2 = np.array([[0.00,  0.00,  -1.00,  0.496], [ 1.00,  0.00,  0.00, -0.743], [0.00,  -1.00, 0.00,  0.555], [ 0.00,  0.00,  0.00,  1.00]])
ikparam = IkParameterization(Tee1[0:3,3],ikmodel2.iktype) # build up the translation3d ik query
# iksolverbase = InterfaceBase.IkSolverBase.Init(robot) I don't know how to use IkSolverBase
handle = manip.GetIkSolver()
sol = manip.FindIKSolution(ikparam,IkFilterOptions.CheckEnvCollisions)
# print "Ik solver:", success # Ik solver: [ 5.55111512e-16 -3.96993549e-12  1.57000000e+00  1.11022302e-16  0.00000000e+00  0.00000000e+00] with manip.GetIkParameterization(IkParameterization.Type.Translation3D)
print "Ik solver:", sol # Ik solver: [ [-1.11022302e-16  8.99614938e-10  1.57000000e+00  1.11022302e-16  0.00000000e+00  0.00000000e+00]] with ikparam
robot.SetDOFValues(sol,ikmodel2.manip.GetArmIndices())
# basemanip.MoveToHandPosition([Tstart],maxiter=1000,maxtries=1,seedik=4)
# robot.WaitForController(0)
raw_input("done")
sys.exit()


# print "testing..."
# successrate, wrongrate = ikmodel2.testik(str(100))
# print "iktest:", successrate, wrongrate
 



    
raw_input('Never here')
sys.exit()

