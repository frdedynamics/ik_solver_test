#!/usr/bin/env python
# -*- coding: utf-8 -*-

# """
# This is the main node calculating robot's joint angles based on given Pose().

# """
import rospy
from Classes.ik_solver_class import IKSolver
from geometry_msgs.msg import Vector3

if __name__ == "__main__": 
	iksolver = IKSolver(START_NODE=True, rate=100)
	iksolver.init_subscribers_and_publishers()
	# pub = rospy.Publisher('/test_msg', Vector3, queue_size=10)
	# rospy.init_node("ik_solver_node")
	start_time = rospy.Time.now()
	rate = rospy.Rate(50)
	test_msg = Vector3()
	while not rospy.is_shutdown():
		current_time = rospy.Time.now()
		print "Elapsed time iksolver:", (current_time-start_time)
		# test_msg.x = 10.0
		# pub.publish(test_msg)
		iksolver.update()
		iksolver.r.sleep()
		# rate.sleep()

# import rospy
# from std_msgs.msg import String

# def talker():
    # pub = rospy.Publisher('chatter', String, queue_size=10)
    # rospy.init_node('talker', anonymous=True)
    # rate = rospy.Rate(10) # 10hz
    # while not rospy.is_shutdown():
        # hello_str = "hello world %s" % rospy.get_time()
        # rospy.loginfo(hello_str)
        # pub.publish(hello_str)
        # rate.sleep()

# if __name__ == '__main__':
    # try:
        # talker()
    # except rospy.ROSInterruptException:
        # pass
