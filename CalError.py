# -*- coding: utf-8 -*-
"""
Created on Tue May  4 11:38:39 2021

@author: dylantan1993@gmail.com

"""

import numpy as np


"""
Taken from EfficientPose whom borrowed from PVN3D

"""



def calc_translation_diff(translation_gt, translation_pred):
    """ Computes the distance between the predicted and ground truth translation
    # Arguments
        translation_gt: numpy array with shape (3,) containing the ground truth translation vector
        translation_pred: numpy array with shape (3,) containing the predicted translation vector
    # Returns
        The translation distance
    """
    return np.linalg.norm(translation_gt - translation_pred)


def calc_rotation_diff(rotation_gt, rotation_pred):
    """ Calculates the distance between two rotations in degree
        copied and modified from https://github.com/ethnhe/PVN3D/blob/master/pvn3d/lib/utils/evaluation_utils.py
    # Arguments
        rotation_gt: numpy array with shape (3, 3) containing the ground truth rotation matrix
        rotation_pred: numpy array with shape (3, 3) containing the predicted rotation matrix
    # Returns
        the rotation distance in degree
    """  
    rotation_diff = np.dot(rotation_pred, rotation_gt.T)
    trace = np.trace(rotation_diff)
    trace = (trace - 1.) / 2.
    if trace < -1.:
        trace = -1.
    elif trace > 1.:
        trace = 1.
    angular_distance = np.rad2deg(np.arccos(trace))
    
    return abs(angular_distance)

#example test
#0000.png
a = [0.7071067363577805, 0.0, 0.7071068260153116, 0.0, 1.0, 0.0, -0.7071068260153116, 0.0, 0.7071067363577805]
a  = np.array(a)
a = np.reshape(a,(3,3))

#0002
b = [0.9063077602437287, 0.0, 0.42261831919829956, 0.0, 1.0, 0.0, -0.42261831919829956, 0.0, 0.9063077602437287]
b  = np.array(b)
b = np.reshape(b,(3,3))



error = calc_rotation_diff(a,b)
print(error)
#this should give 20 degrees difference!