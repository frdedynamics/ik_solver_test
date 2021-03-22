#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
This is the main node calculating the human pose.

"""
# imports
import rospy
import time
from Classes.IMU_subscriber_class_v2 import IMUsubscriber
import Classes.body as body
from sensor_msgs.msg import JointState

joint_pos_kf = []
joint_angles_kf = JointState()
IMU = IMUsubscriber()
index = 0


def human_init():
    joint_angles_kf.name = ['left_shoulder_2', 'left_shoulder_0', 'left_shoulder_1', 'left_elbow_2', 'left_elbow_0', 'left_elbow_1', 'left_wrist_2', 'left_wrist_0', 'left_wrist_1']
    joint_angles_kf.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]


if __name__ == "__main__":
    # Create initial readings and pose_init_shoulder
    human_init()
    IMU.init_subscribers_and_publishers()
    body.initialize_pose(IMU)
    start = time.time()
    prev = 0
    prev2 = 0
    # rate = rospy.Rate(50)

    while not rospy.is_shutdown():
        now = time.time()
        # print "++++", index
        index += 1
        # print "total: ", (now - start)
        # print "interval:", (now - prev), "calibration_flag:", IMU.calibration_flag
        if IMU.calibration_flag < 20:
            IMU.update2(joint_angles_kf)
            IMU.r.sleep()
            prev = now
            index = 0
        else:
            if (index == 5):
                joint_pos_kf = body.update_pose(IMU)
                joint_angles_kf.position = body.calculate_joint_angles(joint_pos_kf)
                index = 0
                # print "kf:", (now - prev2), "calibration_flag:", IMU.calibration_flag
                prev2 = now
            IMU.update2(joint_angles_kf)
            IMU.r.sleep()
            prev = now
