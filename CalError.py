# -*- coding: utf-8 -*-
"""
Created on Tue May  4 11:38:39 2021

@author: dylantan1993@gmail.com

"""

import numpy as np


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
    """ Calculates the distance between two rotational matrices
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


if __name__ == '__main__':
# example test
# two matrices that should be approximately 20 degrees apart
    a = np.array([[0.7071067363577805, 0.0, 0.7071068260153116], 
                  [0.0,                1.0, 0.0               ],
                  [-0.7071068260153116, 0.0, 0.7071067363577805]])

    b = np.array([[0.9063077602437287, 0.0, 0.42261831919829956],
              [0.0, 1.0, 0.0],
              [-0.42261831919829956, 0.0, 0.9063077602437287]])

    error = calc_rotation_diff(a,b)
    print(error)
    #this should give 20 degrees difference!