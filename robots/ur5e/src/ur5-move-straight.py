
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



with env:
  ikmodel = databases.inversekinematics.InverseKinematicsModel(robot=robot,iktype=IkParameterization.Type.Transform6D)
  if not ikmodel.load():
    ikmodel.autogenerate()
  basemanip = interfaces.BaseManipulation(robot)
  taskmanip = interfaces.TaskManipulation(robot)
  robot.SetJointValues([-0.97],ikmodel.manip.GetGripperIndices())
  robot.SetDOFValues([0.0,0.0,1.57,0.0,0.0,0.0])
  Tstart = np.array([[0.0,  0.0007, -0.999999683,  0.496755774],
 [ 1.00000000,  0.0,  0.0, -0.743859000],
 [ 0.0, -0.999999683, 0.0,  0.554908046],
 [ 0.00000000,  0.00000000,  0.00000000,  1.00000000]])
  # Tstart = np.array([[1.0,  0.0,  0.0,  1.18],
										 # [0.0,  1.0,  0.0, -0.74],
										 # [0.0,  0.0,  0.0,  1.00],
										 # [0.0,  0.0,  0.0,  1.00]])
  # Tstart = np.array([[-4.89630558e-12,  1.00000000e+00,  9.79261117e-12,  1.18430000e+00],
 # [ 1.00000000e+00,  4.89630558e-12,  0.00000000e+00, -7.43859000e-01],
 # [-4.79473950e-23,  9.79261117e-12, -1.00000000e+00,  1.01160000e+00],
 # [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00]])
  # Tstart = np.array([[ -1,  0,  0,   2.00000000e-01], [  0,0,   1, 6.30000000e-01], [  0,   1  , 0,   5.50000000e-02], [  0,0,0,1]])
  sol = ikmodel.manip.FindIKSolution(Tstart,IkFilterOptions.CheckEnvCollisions)
  robot.SetDOFValues(sol,ikmodel.manip.GetArmIndices())
basemanip.MoveToHandPosition([Tstart],maxiter=1000,maxtries=1,seedik=4)
robot.WaitForController(0)

print "Active DOF values:", robot.GetActiveDOFValues()
Tee = manip.GetEndEffectorTransform() # get end effector
print "Tee:", Tee
h = env.plot3(Tee[0:3,3],20) # plot one point -- makes pink mark

target = env.GetKinBody('cylinder_green_3')
print "cylinder pos:", target.GetTransform ()
# cylinder pos: [[-1.00000000e+00 -2.46519033e-32  1.22464680e-16  2.00000000e-01]
 # [ 1.22464680e-16 -2.22044605e-16  1.00000000e+00  6.20000000e-01]
 # [ 0.00000000e+00  1.00000000e+00  2.22044605e-16  2.90000000e-02]
 # [ 0.00000000e+00  0.00000000e+00  0.00000000e+00  1.00000000e+00]]
updir = np.array((0,0,1))
success = basemanip.MoveHandStraight(direction=updir,stepsize=0.01,minsteps=1,maxsteps=40)
robot.WaitForController(0)

# T = ikmodel.manip.GetTransform()
# T[1,3] -= 0.1
# success = basemanip.MoveHandStraight(direction=updir,starteematrix=T,stepsize=0.01,minsteps=1,maxsteps=20)
# robot.WaitForController(0)
    

print 'checking for existance of trajectories with random queries of moving in a straight line'
armlength = 0
armjoints = [j for j in robot.GetDependencyOrderedJoints() if j.GetJointIndex() in ikmodel.manip.GetArmIndices()]
eetrans = ikmodel.manip.GetTransform()[0:3,3]

print "eetrans:", eetrans

for j in armjoints[::-1]:
  armlength += sqrt(sum((eetrans-j.GetAnchor())**2))
  eetrans = j.GetAnchor()
stepsize=0.01
failedattempt = 0

while True:
  with env:
    #Tee = dot(ikmodel.manip.GetTransform(),matrixFromAxisAngle(random.rand(3)-0.5,0.2*random.rand()))
    Tee = matrixFromAxisAngle(np.random.rand(3)-0.5,pi*np.random.rand())
    direction = np.random.rand(3)-0.5
    direction /= np.linalg.norm(direction)
    x = np.random.rand(3)-0.5
    length = 0.6*np.random.rand()*armlength
    Tee[0:3,3] = eetrans + x/np.linalg.norm(x)*(armlength-length)
    maxsteps=int(length/stepsize)+1
    minsteps = maxsteps/2
    h = env.drawlinelist(np.array([Tee[0:3,3],Tee[0:3,3]+direction*maxsteps*stepsize]),1)
  try:
    success = basemanip.MoveHandStraight(direction=direction,starteematrix=Tee,stepsize=stepsize,minsteps=minsteps,maxsteps=maxsteps)
    params = (direction,Tee)
    print '%d failed attemps before found'%failedattempt,repr(params)
    failedattempt = 0
    h = env.drawlinelist(np.array([Tee[0:3,3],Tee[0:3,3]+direction*maxsteps*stepsize]),4,[0,0,1])
    robot.WaitForController(0)
      
  except planning_error,e:
    failedattempt += 1
    
raw_input('Never here')
sys.exit()

