# -*- coding: utf-8 -*-
"""
Created on Tue Feb 23 22:48:43 2021

@author: Dylan Tan
"""

import math
import numpy as np


def euclidean(a,b,axis=3):
    output = [pow((a[j]-b[j]),2) for j in range(axis)]
    output = math.sqrt(sum(output))
    return output


def getRmatrix(roll,pitch,yaw):
    
    """
    Input: roll, pitch and yaw, in radians
    Returns: 3 x 3 Rotation matrix
    """
    
    yawMatrix = np.matrix([[math.cos(yaw), -math.sin(yaw), 0],
                           [math.sin(yaw), math.cos(yaw), 0],
                           [0, 0, 1]])

    pitchMatrix = np.matrix([[math.cos(pitch), 0, math.sin(pitch)],
                             [0, 1, 0],
                             [-math.sin(pitch), 0, math.cos(pitch)]])

    rollMatrix = np.matrix([[1, 0, 0],
                            [0, math.cos(roll), -math.sin(roll)],
                            [0, math.sin(roll), math.cos(roll)]])
    
    R = yawMatrix * pitchMatrix * rollMatrix
    
    return R

def rmat2rpy(R) :
    """
    Parameters
    ----------
    R : (3,3) np.array
        Takes in (3,3) numpy array

    Returns
    -------
    Roll pitch and yaw in radians
    """

    sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])

    singular = sy < 1e-6

    if  not singular :
        x = math.atan2(R[2,1] , R[2,2])
        y = math.atan2(-R[2,0], sy)
        z = math.atan2(R[1,0], R[0,0])
    else :
        x = math.atan2(-R[1,2], R[1,1])
        y = math.atan2(-R[2,0], sy)
        z = 0

    return np.array([x, y, z])

def rmat2rot(rmat):
    """
    Parameters
    ----------
    rmat : (3,3) np array
        3 by 3 rotation matrix

    Returns
    -------
    Rotation vector where the W vector is the normalized magnitude of the 3 vectors
    read more on the UR10's rotational vector expression if necessary.'
    """
    
    R = rmat
    theta = math.acos(((R[0, 0] + R[1, 1] + R[2, 2]) - 1) / 2)
    multi = 1 / (2 * math.sin(theta))

    
    rx = multi * (R[2, 1] - R[1, 2]) * theta
    ry = multi * (R[0, 2] - R[2, 0]) * theta
    rz = multi * (R[1, 0] - R[0, 1]) * theta
    outputrotationvector = [rx,ry,rz]

    return outputrotationvector

def rpy2rot(vector):
    """
    Takes in anchor vector of length 9
    Roll, pitch, yaw to rotation vector (UR10 version)
    """
    roll = vector[6]
    pitch = vector[7]
    yaw = vector[8]
    R = getRmatrix(roll,pitch,yaw)

    theta = math.acos(((R[0, 0] + R[1, 1] + R[2, 2]) - 1) / 2)
    multi = 1 / (2 * math.sin(theta))

    
    rx = multi * (R[2, 1] - R[1, 2]) * theta
    ry = multi * (R[0, 2] - R[2, 0]) * theta
    rz = multi * (R[1, 0] - R[0, 1]) * theta
    outputrotationvector = [rx,ry,rz]

    return outputrotationvector


def unit(inpt):
    """
    takes vector and outputs Unit vector 
    """


    length=0
    for i in inpt:
        
        length += (i**2)
    output = inpt/math.sqrt(length)
    return output

def rotateonspot( mat , axis, theta):
    """
    Takes in an input (3,3) matrix, an axis(3 row vector) to rotate around and the angle to rotate it by in degrees
    """
    
    identity = np.eye(3)
    
    
    # zaxis = np.array([0,0,1]) #using z axis

    
    rad = (3.142*theta)/180
    
    W = np.array([[0, -axis[2], axis[1]],
                [axis[2], 0, -axis[0]],
                [-axis[1], axis[0], 0]])

    rod = identity + np.sin(rad)*W + (1-np.cos(rad)) * np.matmul(W,W)
    newv = np.matmul(rod,mat)
    
    return newv

if __name__ == '__main__':
    
    print("Testing all functions: ")
    print("euclidean(): ",euclidean([1,1,1],[2,2,2],3))
    print("getRmatrix:",getRmatrix(1.57,1.57,1.57))
    print("rpy2rot: ",rpy2rot([0,0,0,0,0,0,1.57,1.57,1.57]))
    print("rotateonspot: ", rotateonspot(np.eye(3),np.array([0,0,1]),30))
    
    
    






