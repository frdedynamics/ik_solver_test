#!/usr/bin/env python
# -*- coding: utf-8 -*-

from openravepy import *
import sys
import numpy as np


# initialize environment
env = Environment()
env.StopSimulation()
# openrave_root = './'
# env.Load(openrave_root + "baxter_ik.xml")
env.Load("sc_3dof.xml")
env.SetViewer('qtcoin') # start the viewer (conflicts with matplotlib)


    
robot = env.GetRobots()[0] # get the first robot

manip = robot.SetActiveManipulator('manipulator') # set the manipulator to right_arm

dummy_joints = [1.57, 0.0, 0.0]
robot.SetDOFValues(dummy_joints, manip.GetArmIndices()) # set the current solution
current_pose = manip.GetEndEffectorTransform()
print current_pose

# # try IK for similar pose
goal_pose = current_pose
# # goal_pose[2,3] -= 0.05
ikmodel = databases.inversekinematics.InverseKinematicsModel(robot,iktype=IkParameterization.Type.Translation3D)
print ikmodel.load()


# sol = manip.FindIKSolution(goal_pose, 18) # get collision-free solution
# print "IK solution:", sol

# ikmodel = databases.inversekinematics.InverseKinematicsModel(robot,iktype=IkParameterization.Type.Translation3D)
# if not ikmodel.load():
    # ikmodel.autogenerate()

index = 0
with env:
		# move the robot in a random collision-free position and call the IK
		while True:
				target=ikmodel.manip.GetTransform()[0:3,3]+(np.random.rand(3)-0.5)
				solutions = ikmodel.manip.FindIKSolutions(IkParameterization(target,IkParameterization.Type.Lookat3D),IkFilterOptions.CheckEnvCollisions)
				print "index:", index
				index += 1
				if len(solutions) > 0:
						break
raw_input('Hit ENTER to continue.')
env.Destroy()

with robot: # lock environment and save robot state
    robot.SetDOFValues([0.0, 0.0, 0.0],[0,1,2]) # set the first 3 dof values
    Tee = manip.GetEndEffectorTransform() # get end effector
    print Tee
    ikparam = IkParameterization(Tee[0:3,3],ikmodel.iktype) # build up the translation3d ik query
    print ikparam
    print ikparam.GetType()
    print ikparam.GetNumberOfValues()
    print ikparam.GetValues()
    print ikparam.GetDOF()
    print ikparam.GetRotation3D()
    print ikparam.GetTranslation3D()
    
    # joint_conf = databases.inversekinematics.RaveCreateIkSolver(env, 'sc_3dof')
    # print "joint_conf", joint_conf
    
    ikmodel = databases.inversekinematics.InverseKinematicsModel(robot, iktype=IkParameterization.Type.Translation3D)
    ikname = ikmodel.getikname()
    iktest = ikmodel.testik()
    print "ikname", ikname

    sys.exit("Done")
    sols = manip.FindIKSolutions(ikparam, IkFilterOptions.CheckEnvCollisions) # get all solutions
    sol = manip.FindIKSolution(goal_pose, 18) # get collision-free solution

print sols
sys.exit("Done")

h = env.plot3(Tee[0:3,3],10) # plot one point
with robot: # save robot state
    for sol in sols[::10]: # go through every 10th solution
        robot.SetDOFValues(sol,manip.GetArmIndices()) # set the current solution
        env.UpdatePublishedBodies() # allow viewer to update new robot
        raw_input('press any key')

raveLogInfo('restored dof values: '+repr(robot.GetDOFValues())) # robot state is restored to original
