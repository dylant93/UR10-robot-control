- networksettings: set this up so that you can control the UR10 via ethernet
- step1: plug these two in and power on
- step2: press power button
- step3: plug in ethernet cable
- step4: press initialize robot
- step5: press start
- step6: check the robot for clearance. make sure theres no body around and that the arm
has ample room to move
- step7: press auto, if there is insufficient room to move the robot, manually move 
it using the green arrows
- step8: When movement all says ok. exit with the okay button at my thumb
- step9: go to setup
- step10: setup network
- step11: verify the ip address and subnet mask. subnet mask MUST be 255.255.25.0
IP address ONLY HAS TO FOLLOW THE IP ADDRESS SPECIFIED IN THE MAIN FILE AT ROBOT CLASS
instantiation (currently 192.168.1.100)
- step11b: double check the code 
- step12: exit with back button
- step13: program rbot
- step14: empty program
- step15: go to installation
- step16: check for tool center. notice it says offset 45mm and 105mm. 
- step17: look at this figure and notice that 45 and 105 are the offsets for the 3d
printed mount. in future change as you deem fit, if you flip, if you redesign. 
You are more or less done with setup
- step18: go to move
- step19: plug camera in, make sure that double USBC cable is plugged into comp, dont mix it up
- step20: run main_capture_step1.py, if all goes well, it should align arm to rest position
shown in the picture.
- step21: click on base on the robot and verify that the numbers in 21a
- step21a: the numbers in green should be 550,0,550. this is the resting position.
FYI: the numbers in pink are joint defined postions this can be verified in step21b
step21b: you can change the numbers as you like, but for now this are the defined absolute
resting position and orientations
step22: place camera mount in
step23: run main_capture_step1 again and proceed with demo