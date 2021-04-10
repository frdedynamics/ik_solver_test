
from openravepy import *
import sys, os
import numpy as np

sys.path.append("/home/gizem/catkin_ws/src/ur5_with_hand_gazebo/src/Classes")
from DH_matrices import DHmatrices

sys.path.append("/home/gizem/catkin_ws/src/ur5_with_hand_gazebo/src/Classes")
robot_dir = "/home/gizem/catkin_ws/src/ik_solver_test/robots/ur5e/xml/"
robot_name = "ur5e.xml"
robot_path = os.path.join(robot_dir, robot_name)


env = Environment() # create the environment
#env.Load('data/lab1.env.xml') # load a scene
# env.Load('planar_3dof.xml') # load a scene
# env.Load('../xml/ur5e.xml') # load a scene
env.Load(robot_path) # load a scene
env.SetViewer('qtcoin') # start the viewer
robot = env.GetRobots()[0] # get the first robot
print "Dof", robot.GetDOFValues()


robot.SetDOFValues([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
dummy_input = raw_input("Change joints")
robot.SetDOFValues([-0.95057231,  2.65834737, -1.22379041, -3.06405115,  1.45200026, -0.88941813])
dummy_input = raw_input("Change joints")
robot.SetDOFValues([-0.95057231,  2.52574301, -1.52847648,  0.5148319,  -1.45200026,  2.25217462])
dummy_input = raw_input("Change joints")
robot.SetDOFValues([ 1.29999983,  2.07419062, -1.5507617,  -1.96502137,  1.60000002, -3.1415925 ])
dummy_input = raw_input("Change joints")
robot.SetDOFValues([ 1.29999983,  1.64518237, -1.20000005,  1.25481772,  -1.60000002,  0.0] )
dummy_input = raw_input("Change joints")
robot.SetDOFValues([-0.95057231,  1.49078465,  1.22379041,  1.939116,    1.45200026, -0.88941813])
dummy_input = raw_input("Change joints")
robot.SetDOFValues([-0.95057231,  1.07405365,  1.52847648, -1.09043157, -1.45200026,  2.25217462])
dummy_input = raw_input("Change joints")
robot.SetDOFValues([ 1.29999983,  0.60194546,  1.5507617,   2.68888545,  1.60000002, -3.1415925 ])
dummy_input = raw_input("Change joints")
robot.SetDOFValues([ 1.29999983,  0.50000000,  1.20000005, 0.0,  -1.60000002,  0.0])
dummy_input = raw_input("Change joints")
sys.exit()


manip = robot.GetActiveManipulator()
Tee = manip.GetEndEffectorTransform() # get end effector
print "Tee:", Tee
Tee_pose = DHmatrices.htm_to_pose(Tee)
print "Tee_pose:", Tee_pose
Tee_np = DHmatrices.pose_to_htm(Tee_pose)
print "Tee_htm:", Tee_np
dummy_input = raw_input("Gone to IKmodel?")
robot.SetDOFValues([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

ikmodel=databases.inversekinematics.InverseKinematicsModel(robot,iktype=IkParameterization.Type.Translation3D)
if not ikmodel.load():
    print "new ik model loading"
    ikmodel.autogenerate()

print "ikmodel file name:", ikmodel.getfilename(), ikmodel.load()
dummy_input = raw_input("Go to Tee1?")

Tee1 = np.array([[0.00,  1.00,  0.00,  0.817], [1.00,  0.00,  0.00, -0.232], [0.00,  0.00, -1.00,  0.062], [0.00,  0.00,  0.00,  1.00]])
print Tee1
ikparam = IkParameterization(Tee[0:3,3], ikmodel.iktype) # build up the translation3d ik query
sol = self.manip.FindIKSolution(ikparam, IkFilterOptions.CheckEnvCollisions)
robot.SetDOFValues(sol, ikmodel.manip.GetArmIndices())
dummy_input = raw_input("Done?")

robot.SetDOFValues([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
sys.exit()



with robot: # lock environment and save robot state
	robot.SetDOFValues([0.0,0.0,0.0,0.0,0.0,0.0])
	manip = robot.GetActiveManipulator()
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
