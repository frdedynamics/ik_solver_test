
from openravepy import *
import sys
import numpy as np
env = Environment() # create the environment
#env.Load('data/lab1.env.xml') # load a scene
# env.Load('planar_3dof.xml') # load a scene
env.Load('ur5-arm-wrist.xml') # load a scene
env.SetViewer('qtcoin') # start the viewer
robot = env.GetRobots()[0] # get the first robot
print "Dof", robot.GetDOFValues()


ikmodel=databases.inversekinematics.InverseKinematicsModel(robot,iktype=IkParameterization.Type.TranslationXY2D)
if not ikmodel.load():
	print "creating new IK"
	ikmodel.autogenerate()
    
print "IK model:", ikmodel.load()
robot.SetDOFValues([0.0,0.0,0.0])
wrist = robot.SetActiveManipulator('wrist')
# arm = robot.SetActiveManipulator('arm')

# print "IK solver for arm:", arm.GetIkSolver()
print "IK solver for wrist:", wrist.GetIkSolver()
raw_input('Here')
sys.exit()



robot.SetDOFValues([0.0,0.0,0.0,0.0,0.0,0.0])
basemanip = interfaces.BaseManipulation(robot)
# Tgoal = np.array([[-1.0, 0.0, 0.0,  0.743],
 # [ 0.0,  0.0, -1.0,  1.623],
 # [ 0.0, -1.0, 0.0, -0.572],
 # [ 0.0,  0.0,  0.0,  1.0]])
T_wrist1 = np.array([[-1.00, 0.0, 0.0,  0.743],
 [ 0.0, -0.999, -0.007,  1.127],
 [ 0.0, -0.007,  0.999, -1.068],
 [ 0.0,  0.0,  0.0,  1.00]])

robot.SetActiveDOFs([3,4,5])
robot.SetActiveDOFs([3,4,5])
sol = ikmodel.manip.FindIKSolution(T_wrist1,IkFilterOptions.CheckEnvCollisions)
robot.SetDOFValues(sol,ikmodel.manip.GetArmIndices())
basemanip.MoveToHandPosition([T_wrist1],maxiter=1000,maxtries=1,seedik=4)


# robot.SetAffineDOF([0, 1, 2])
raw_input('Here')
sys.exit()
Tee = manip.GetEndEffectorTransform() # get end effector
print "Tee:", Tee
ikparam = IkParameterization(Tee,ikmodel.iktype) # build up the translation3d ik query
sols = manip.FindIKSolutions(ikparam, IkFilterOptions.CheckEnvCollisions) # get all solutions
print "# of active DOF:", robot.GetActiveDOF()
print "active DOF Indices:", robot.GetActiveDOFIndices()
print "Affine DOF:", robot.GetAffineDOF()
# Tee: [[-9.99998732e-01  1.59265292e-03  1.57534576e-16  1.51116443e-02]
 # [-1.56775634e-16  5.55111512e-16 -1.00000000e+00  2.50000000e-02]
 # [-1.59265292e-03 -9.99998732e-01 -4.44089210e-16 -1.40199037e-01]
 # [ 0.00000000e+00  0.00000000e+00  0.00000000e+00  1.00000000e+00]]

h = env.plot3(Tee[0:3,3],10) # plot one point -- makes pink mark
Tee = manip.GetEndEffectorTransform() # get end effector
print "Tee:", Tee

with robot: # save robot state
    for sol in sols[::10]: # go through every 10th solution
        robot.SetDOFValues(sol,manip.GetArmIndices()) # set the current solution
        env.UpdatePublishedBodies() # allow viewer to update new robot
        raw_input('press any key')
raveLogInfo('restored dof values: '+repr(robot.GetDOFValues())) # robot state is restored to original



manipprob = interfaces.BaseManipulation(robot) # create the interface for basic manipulation programs
# Tgoal = numpy.array([4.05000000e-01, 2.50000000e-02, 0.0])
Tgoal = numpy.array([[0.0,  1.0,  0.0,  1.18],
										 [ 1.0,  0.0,  00.0, -0.74],
										 [0.0,  0.0, -1.0,  1.01],
										 [ 0.0,  0.0,  0.0,  1.00]])
# res = manipprob.MoveToHandPosition(matrices=[Tgoal],seedik=10) # call motion planner with goal joint angles
robot.WaitForController(0) # wait
print "Dof", robot.GetDOFValues()
raw_input('Did it move?')

Thand = robot.GetActiveManipulator().GetEndEffectorTransform()
ikmodel2=databases.inversekinematics.InverseKinematicsModel(robot,iktype=IkParameterization.Type.Translation3D)
if not ikmodel2.load():
	print "IK autogenerate"
	ikmodel2.autogenerate()
    
print ikmodel2.load()

print "Thand:", Thand[0:3,3]
Thand[2,3] -= 0.1
print "Thand:", Thand[0:3,3]
sys.exit()

Thand[2,3] -= 0.1
# Thand[2,3] += 0.1
# Thand[0,2] = 0.2
# Thand[1,2] = 0.1
res = manipprob.MoveToHandPosition(translation=Thand[0:3,3])
#res = manipprob.MoveManipulator(goal=[-0.75,0.84,-0.64]) # call motion planner with goal joint angles
robot.WaitForController(0) # wait
raw_input('Did hand move?')
env.Destroy()

Thand = robot.GetActiveManipulator().GetEndEffectorTransform()
ikparam = IkParameterization(Thand[0:3,3],ikmodel.iktype) # build up the translation3d ik query
#Thand[0,2] = 0.2
#Thand[1,2] = 0.1
print ikparam.GetDOF()
raw_input('Hit ENTER to continue.')

#res = manipprob.MoveToHandPosition([Thand])
Thand[0,3] -= 0.1
# Thand[2,3] += 0.1
res = manipprob.MoveToHandPosition(translation=Thand[0:3,3])
#res = manipprob.MoveManipulator(goal=[-0.75,0.84,-0.64]) # call motion planner with goal joint angles
robot.WaitForController(0) # wait

raw_input('Hit ENTER to continue.')
env.Destroy()




