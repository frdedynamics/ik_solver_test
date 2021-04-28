#!/usr/bin/env python
# -*- coding: utf-8 -*-

# """
# This is the main node calculating robot's joint angles based on given Pose().

# """
import rospy
from Classes.ik_solver_class_ur5e import IKSolver
from geometry_msgs.msg import Vector3

if __name__ == "__main__":
	iksolver = IKSolver(START_NODE=True, rate=100)
	iksolver.init_subscribers_and_publishers()
	start_time = rospy.Time.now()
	rate = rospy.Rate(50)
	test_msg = Vector3()
	while not rospy.is_shutdown():
		current_time = rospy.Time.now()
		# print "Elapsed time iksolver:", (current_time-start_time)
		iksolver.update()
		iksolver.r.sleep()
