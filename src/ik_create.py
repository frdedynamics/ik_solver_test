#!/usr/bin/env python

"""
Test purpose. Not working. env and ikfast cannot be imported.

# kinbody = env.ReadRobotXMLFile(robot_path)
# env.Add(kinbody)
# solver = ikfast.IKFastSolver(kinbody=kinbody)
# chaintree = solver.generateIkSolver(baselink=0,eelink=7,freeindices=[2],solvefn=ikfast.IKFastSolver.solveFullIK_6D)
# code = solver.writeIkSolver(chaintree)
# open('ik.cpp','w').write(code)
"""

from openravepy import *
import sys, os

sys.path.append("/home/gizem/catkin_ws/src/ur5_with_hand_gazebo/src/Classes")
robot_dir = "/home/gizem/catkin_ws/src/ik_solver_test/robots/ur5e/xml/"
robot_name = "ur5e.robot.xml"
robot_path = os.path.join(robot_dir, robot_name)


env = Environment() # create the environment
# env.Load(robot_path) # load a scene
kinbody = env.ReadRobotXMLFile(robot_path)
env.Add(kinbody)
env.SetViewer('qtcoin') # start the viewer
dummy_input = raw_input()
# robot = env.GetRobots()[0] # get the first robot
# print "Dof", robot.GetDOFValues()

# robot.SetDOFValues([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
