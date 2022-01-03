# -*- coding: utf-8 -*-
"""
Created on Wed Feb 24 17:47:58 2021

@author: holy_
"""
from matplotlib import pyplot as plt


def extractlog(log):
    sortedlog = []
    for j in range(log.shape[1]):
        temp = [i[j] for i in log]
        sortedlog.append(temp)
        
    # x = [i[0] for i in log]
    # y = [i[1] for i in log]
    # z = [i[2] for i in log]
    # sortedlog = [x,y,z]
    
    return sortedlog
    

def plotxy(sortedlog):
    
    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.set_aspect('equal', adjustable='box')
    ax.set_xlabel('X ')
    ax.set_ylabel('Y ')
    ax.scatter(sortedlog[0],sortedlog[1])
    return

def plotyz(sortedlog):
    
    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.set_aspect('equal', adjustable='box')
    ax.set_xlabel('Y ')
    ax.set_ylabel('Z ')
    ax.scatter(sortedlog[1],sortedlog[2])
    return

def plotxz(sortedlog):
    
    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.set_aspect('equal', adjustable='box')
    ax.set_xlabel('X ')
    ax.set_ylabel('Z ')
    ax.scatter(sortedlog[0],sortedlog[2])
    return

def plot3d(sortedlog):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(sortedlog[0], sortedlog[1], sortedlog[2], c='r', marker='o')
    ax.set_xlabel('X ')
    ax.set_ylabel('Y ')
    ax.set_zlabel('Z ')
    plt.show()
    
# def plotrpy(anch):
#     temp = getRmatrix(anch[6],anch[7],anch[8])
#     normalv = np.array([[0],[0],[1]])
#     R = temp * normalv
#     x,y,z = anch[:3]
#     v = [R[0,0]*0.1+x,R[1,0]*0.1+y,R[2,0]*0.1+z]
#     logscatter(log, v)
#     v = [R[0,0]*0.2+x,R[1,0]*0.2+y,R[2,0]*0.2+z]
#     logscatter(log, v)
        
#     return

# def plotRvector(anch):
#     v = rpy2rot(anch)
#     theta = euclidean(v, [0,0,0])
#     v = unit(np.array(v))
#     identity = np.eye(3)
#     zaxis = np.array([0,0,1])
#     W = np.array([[0, -v[2], v[1]],
#                        [v[2], 0, -v[0]],
#                        [-v[1], v[0], 0]])
        
#     rod = identity + np.sin(theta)*W + (1-np.cos(theta)) * np.matmul(W,W)
#     output = np.matmul(rod,zaxis)
#     x,y,z = anch[:3]
#     logscatter(log, anch[:3])
#     v = [output[0]*0.1+x,output[1]*0.1+y,output[2]*0.1+z]
#     logscatter(log, v)
#     v = [output[0]*0.2+x,output[1]*0.2+y,output[2]*0.2+z]
#     logscatter(log, v)
#     return
    