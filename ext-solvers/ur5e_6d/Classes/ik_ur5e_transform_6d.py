#!/usr/bin/env python

"""
Created using ikfastpy: 
*the `ikfast61.cpp` is created: `openrave.py --database inversekinematics --robot=ur5e.xml --iktype=transform6d --precision=4`* where ur5e.xml: /home/gizem/catkin_ws/src/ik_solver_test/robots/ur5e/xml
"""
import os, sys
import numpy as np
from math import pi
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

    
    def calc_inverse_kin(self, ee_coord):
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
                return self.joint_configs
        except AssertionError as e:
            print e
    

    def choose_closest_soln(self, current_angles, restricted=True):
        diff = []
        for joint_config in self.joint_configs:
            diff.append(mse(joint_config, current_angles))
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
                sys.exit()
            else:
                diff_base = np.ones(3)
                for i in range(3):
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


    
    def apply_joint_limits(self, current_angles, **kwargs):
        ''' Manually apply joint limits LATER
            Call this function like: iksolverwhatever.apply_joint_limits(shoulder_pan=angle_x, shoulder_lift=angle_y, wrist_3=angle_z)'''
        for joint, theta in kwargs.items():
            joint_int = IK_UR5ETRANSFORM6D.joint_names_to_numbers(joint)
            if joint_int == 0:
                pass
            elif joint_int == 1:
                pass
            else:
                sys.exit("Unknown joint name or limit")


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

