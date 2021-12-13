# -*- coding: utf-8 -*-
"""
Created on Thu Jun  3 01:52:50 2021

@author: dylantan1993@gmail.com

"""
import open3d as o3d
import json

reader = o3d.io.AzureKinectMKVReader()

reader.open("nfov.mkv")
metadata = reader.get_metadata()

o3d.io.write_azure_kinect_mkv_metadata('nfov.json', metadata)
# print(metadata)

