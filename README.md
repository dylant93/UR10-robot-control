## UR 10 Demo and control 
Most of the important files are here in the main directory.
i'll briefly go through the files here. More info can be found inside the 
individual files themselves.

#Demo

For the demo, theres only 3 main files.
main_Capture_step1
main_Verify_step2
main_Move_step3

This should give you the required information to complete the demo.

Some various other files are the original 
main.py file - its just the 3 steps more or less combined.
open3dfunctions - contains various useful functions of o3d and also running the 
	it would call a manual registration which could be useful
capturefast - will come in very handy when you, well want to capture images fast
visualization - is a supporting class used in verify_step2
robot - is the main ur10 class 
camera - is the main class that uses pyk4a for kinect azure
geometry - contains some useful transformation functions used in the various classes
plot - is sometimes useful to plot the coordinates when planning a robot movement