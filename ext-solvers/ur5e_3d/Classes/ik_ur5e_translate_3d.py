#!/usr/bin/env python

"""
Created using ikfastpy: 
python `openrave-config --python-dir`/openravepy/_openravepy_/ikfast.py --robot=ur5e.robot.xml --iktype=translation3d --baselink=3 --eelink=6 --savefile=ikfast31.cpp --maxcasedepth 1

"""

import numpy as np
import ikfastpy
from math import pi
from sklearn.metrics import mean_squared_error as mse
import sys


class IK_UR5ETRANS3D:
    def __init__(self):
        self.kinematics = ikfastpy.PyKinematics()
        self.n_joints = self.kinematics.getDOF()
        self.joint_angles = [0.1, 0.1, 0.1]
        self.joint_configs = []
        self.n_solutions = 0
        self.ee_pose = [0.] * 12
        self.ee_coord = [0., 0., 0.]  # update initial with a valid
        print "Initialized"


    def calc_forward_kin(self, joint_angles):
        '''
        Given list of joint angles in radians, return ee coordinates
        @params: joint_angles=[j1, j2, j3]
        @returns: ee=[x, y, z]
        '''
        print("\nComputing forward kinematics:\n")
        self.ee_pose = self.kinematics.forward(joint_angles)
        self.ee_coord = [self.ee_pose[3], self.ee_pose[7], self.ee_pose[11]]
        return np.asarray(self.ee_coord)

    
    def calc_inverse_kin(self, ee_coord):
        '''
        Given coordinates ee as a list, return all possible joint angles
        @params: ee_coord=[x, y, z]
        @returns: joint_angles=[j1, j2, j3]
        '''
        print("\nComputing inverse kinematics:\n")
        ee_trans = [0., 0., 0., ee_coord[0],
                    0., 0., 0., ee_coord[1],
                    0., 0., 0., ee_coord[2]]
        try:
            self.joint_configs = self.kinematics.inverse(ee_trans)
            self.n_solutions = int(len(self.joint_configs)/self.n_joints)
            # print("%d solutions found:"%(n_solutions))
            self.joint_configs = np.asarray(self.joint_configs).reshape(self.n_solutions,self.n_joints)
            if not self.n_solutions > 0:
                raise AssertionError("No solution found")
            else:
                return self.joint_configs
        except AssertionError as e:
            print e
    

    def choose_closest_soln(self, current_angles):
        diff = []
        for joint_config in self.joint_configs:
            diff.append(mse(joint_config, current_angles))
        return self.joint_configs[diff.index(min(diff))]
