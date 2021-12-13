# -*- coding: utf-8 -*-
"""
Created on Thu Jun 17 17:42:28 2021

@author: dylantan1993@gmail.com

"""
from camera import mykinectazure
from robot import robot
import os
# import sys, getopt
#PSA 




path = 'temp\\'

if(not os.path.isdir(path)):
    print("No temp folder, creating it") 
    os.mkdir(path)
if(not os.path.isdir(path+"rgb")):
    print("No rgb folder, creating it") 
    os.mkdir(path+"rgb")
if(not os.path.isdir(path+"npy")):    
    print("No npy folder, creating it") 
    os.mkdir(path+"npy")
if(not os.path.isdir(path+"depth")):  
    print("No depth folder, creating it") 
    os.mkdir(path+"depth")
if(not os.path.isdir(path+"depthvisual")):  
    print("No depthvisual folder, creating it") 
    os.mkdir(path+"depthvisual")
    
print("All folders are intact.. proceeding")


li = os.listdir(path+"rgb") # dir is your directory path
number_files = len(li)


robot2 = robot(connection=False,homedistance = 1000)
camera = mykinectazure(connection=True,namecounter=number_files)
camera.path = path
conetype = 0


print("\nCapturing image",number_files, '\n..\n..\nPlease wait...\n..\n..')
camera.cap()
camera.saveupdate(visual = True)
# number_files+=1
    
print("End")




    