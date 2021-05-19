#!/usr/bin/env python

"""
Created using ikfastpy: 
*the `ikfast61.cpp` is created: `openrave.py --database inversekinematics --robot=ur5e.xml --iktype=transform6d --precision=4`* where ur5e.xml: /home/gizem/catkin_ws/src/ik_solver_test/robots/ur5e/xml
"""
import os, sys
import numpy as np
from math import pi
from math import radians as d2r
from sklearn.metrics import mean_squared_error as mse

sys.path.append("/home/gizem/catkin_ws/src/ik_solver_test/ext-solvers/ur5e_6d")
import ikfastpy



class IK_UR5ETRANSFORM6D:
    def __init__(self):
        self.kinematics = ikfastpy.PyKinematics()
        self.n_joints = self.kinematics.getDOF()
        print "# of joints:", self.n_joints
        self.joint_angles = [0.1,-0.75,0.2,1.5,-0.6,0.] # in radians
        self.joint_configs = []
        self.n_solutions = 0
        self.ee_pose = [0.] * 12
        self.ee_coord = [0., 0., 0.]  # update initial with a valid
        self.closest_soln = []

        print "Initialized"


    def calc_forward_kin(self, joint_angles):
        '''
        Given list of joint angles in radians, return ee coordinates
        @params: joint_angles=[j1, j2, j3, j4, j5, j6]
        @returns: ee=[x, y, z, theta1, theta2, theta3]
        '''
        print("\nComputing forward kinematics:\n")
        self.ee_pose = self.kinematics.forward(joint_angles)
        self.ee_coord = [self.ee_pose[3], self.ee_pose[7], self.ee_pose[11]]
        return np.asarray(self.ee_coord)

    
    def calc_inverse_kin(self, ee_coord, limitted=True):
        '''
        Given coordinates ee as a list, return all possible joint angles
        @params: ee_coord=(3,4) np.ndarray
        @returns: joint_angles=[j1, j2, j3, j4, j5, j6]
        '''
        # print("\nComputing inverse kinematics:\n")
        if(ee_coord.shape == (4,4)):
            ee = ee_coord[:3,:]
        elif(ee_coord.shape == (3,4)):
            ee = ee_coord
        else:
            raise AssertionError("Wrong type of ee. Expected (3,4) or (4,4) np.ndarray")

        try:
            self.joint_configs = self.kinematics.inverse(ee.reshape(-1).tolist())
            self.n_solutions = int(len(self.joint_configs)/self.n_joints)
            # print("%d solutions found:"%(n_solutions))
            self.joint_configs = np.asarray(self.joint_configs).reshape(self.n_solutions,self.n_joints)
            if not self.n_solutions > 0:
                raise AssertionError("No solution found")
            else:
                if limitted:
                    print "sols before limit:", self.joint_configs.shape
                    # self.apply_joint_limits(shoulder_pan=[-pi/4, pi/4], shoulder_lift=[-3*pi/4, -pi/4], wrist_2=[-pi, -pi/2])
                    self.apply_joint_limits(shoulder_lift=[d2r(-45.0), 0.5])
                    print "sols after limit:", self.joint_configs.shape
                    # print "joint configs after:", self.joint_configs
                    self.n_solutions = len(self.joint_configs)
                    print "n_solutions after:", self.n_solutions
                return self.joint_configs
        except AssertionError as e:
            print e
    

    def choose_closest_soln(self, current_angles, restricted=False):
        diff = []
        for joint_config in self.joint_configs:
            diff.append(mse(joint_config, current_angles))
            # diff.append(mse(joint_config, current_angles, sample_weight=[10.0, 0.01, 10.0, 0.01, 10.0, 0.01]))
        closest_soln_index = diff.index(min(diff))
        self.closest_soln = self.joint_configs[closest_soln_index]
        print "closest:", self.closest_soln
        print "current:", current_angles
        if restricted:
            changed = self.eliminate_jumps(current_angles, self.closest_soln, closest_soln_index)
            while changed:  # which means there is a jump on the base joint
                self.choose_closest_soln(current_angles, restricted=restricted)
                changed = self.eliminate_jumps(current_angles, self.closest_soln, closest_soln_index)
        else:
            pass
        return self.closest_soln


    def eliminate_jumps(self, current_angles, closest_soln, closest_soln_index):
        try:
            print "self.n_solutions:", self.n_solutions
            if not self.n_solutions > 1:
                changed = False
                raise AssertionError("All solutions removed")
            else:
                diff_base = np.ones(6)
                for i in range(6):
                    diff_base[i] = abs(current_angles[i]-closest_soln[i])
                    diff_base[i] = True if diff_base[i] > 0.707 else False
                # diff_base = current_angles[1]-closest_soln[1]
                print "diff_base:", diff_base
                if np.any(diff_base):
                    self.joint_configs = np.delete(self.joint_configs, closest_soln_index, 0)
                    print "Removed soln:", closest_soln
                    # print "Current angles:", current_angles
                    self.n_solutions = self.n_solutions -1
                    changed = True
                else:
                    print "No need to chose new soln"
                    changed = False
            return changed
        except AssertionError as e:
            print e


    
    def apply_joint_limits(self, **kwargs):
        ''' Manually apply joint limits
            Call this function like: iksolverwhatever.apply_joint_limits(shoulder_pan=[theta_min, theta_max], shoulder_lift=angle_y, wrist_3=angle_z)'''
        remove_index_list = []
        for joint, theta in kwargs.items():
            joint_int = IK_UR5ETRANSFORM6D.joint_names_to_numbers(joint)
            # print "joint:", joint, "theta:", theta
            for sol_index in range(self.n_solutions):
                if not ((self.joint_configs[sol_index, joint_int] > theta[0]) and (self.joint_configs[sol_index, joint_int] < theta[1])):
                    remove_index_list.append(sol_index)

        # remove_index_list = []
        # for sol_index in range(self.n_solutions):
        #     if not ((self.joint_configs[sol_index, 0] > 0.0) and (self.joint_configs[sol_index, 0] < 3*pi/4)):
        #         remove_index_list.append(sol_index)

        print "n of sols before:", len(self.joint_configs/self.n_joints)
        remove_index_list = list(set(remove_index_list))
        remove_index_list.sort(reverse=True)  
        print "to be removed index:", remove_index_list
        for r in range(len(remove_index_list)):
            # print "shape:", self.joint_configs.shape
            # print "to be removed:", remove_index_list[r]
            # print "joints:", self.joint_configs[remove_index_list[r]]
            # dummy = raw_input()
            print "Removed:", self.joint_configs[remove_index_list[r], 0], "for joint:", joint, joint_int, "limits:", theta
            self.joint_configs = np.delete(self.joint_configs, remove_index_list[r], 0)
            # print "r:", r
                

    @staticmethod
    def joint_names_to_numbers(argument): 
        switcher = { 
            "shoulder_pan": 0, 
            "shoulder_lift": 1,
            "elbow": 2,
            "wrist_1": 3,
            "wrist_2": 4,
            "wrist_3": 5,
        }
        # get() method of dictionary data type returns  
        # value of passed argument if it is present  
        # in dictionary otherwise second argument will 
        # be assigned as default value of passed argument 
        return switcher.get(argument, "nothing")

