"""Shows how to get all 6D IK solutions.
Derived from http://openrave.org/docs/latest_stable/tutorials/openravepy_examples/
"""
from openravepy import *
import numpy as np
import time, sys
# openrave_path = '/home/gizem/Programs/OpenRAVE/repos/openrave/src/data/'

env = Environment() # create the environment
env.SetViewer('qtcoin') # start the viewer
# env.Load('../xml/ur5-vertical.xml') # load a scene
env.Load('../xml/ur5-with-objects.xml') # load a scene
robot = env.GetRobots()[0] # get the first robot
print robot.GetActiveDOF()


# manip = robot.SetActiveManipulator('leftarm_torso') # set the manipulator to leftarm
manip = robot.GetActiveManipulator()
ikmodel = databases.inversekinematics.InverseKinematicsModel(robot,iktype=IkParameterization.Type.Transform6D)

print "ikmodel file:", ikmodel.getfilename()
print "ikmodel load:", ikmodel.load()


raw_input("Done")
sys.exit()


if not ikmodel.load():
    ikmodel.autogenerate()
    


with env: # lock environment
    Tgoal = np.array([[-1.00, 0.0, 0.0,  0.743], [ 0.0, -0.999, -0.007,  1.127], [ 0.0, -0.007,  0.999, -1.068], [ 0.0,  0.0,  0.0,  1.00]])
    sol = manip.FindIKSolution(Tgoal, IkFilterOptions.CheckEnvCollisions) # get collision-free solution
    with robot: # save robot state
        robot.SetDOFValues(sol,manip.GetArmIndices()) # set the current solution
        Tee = manip.GetEndEffectorTransform()
        env.UpdatePublishedBodies() # allow viewer to update new robot
        time.sleep(10)
    
    raveLogInfo('Tee is: '+repr(Tee))
