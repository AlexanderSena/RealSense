#!/usr/bin/env python

import rospy
import os
import time
import math
import message_filters
import numpy as np
import transforms3d
from geometry_msgs.msg import TransformStamped


def callback(par, will):

	#take in the positions
	p = par.transform.translation
	w = will.transform.translation

	#distances between subject/drone
	x_d = p.x - w.x
	y_d = p.y - w.y
	z_d = p.z - w.z


	#Spatial vector (3D)
	dist = math.sqrt(x_d**2 + y_d**2 + z_d**2)
	u3 = [x_d/dist, y_d/dist, z_d/dist] 
	

	#Spatial vector (2D), used for approx.
	mag2 = math.sqrt(x_d**2 + y_d**2)
	unit = [x_d/mag2, y_d/mag2]
	
	#adjust the volume to decrease as 1/(1+R) from subject
	#Done so that at R = 0, vol = 100%
	vol = 100/(1+dist)


	#transforms vicon rotations into Euler using transforms3d library
	qs = [will.transform.rotation.x, will.transform.rotation.y, will.transform.rotation.z, will.transform.rotation.w]
	Eul = transforms3d.euler.quat2euler(qs)

	#used for checking degree values during sanity checks
	#Eul = [(180/np.pi)*Eul[0], (180/np.pi)*Eul[1], (180/np.pi)*Eul[2]]

	#The angle (deg) in between heading and the drone
	ang = (180/np.pi)*math.acos(unit[0]*math.cos(Eul[0])+unit[1]*math.sin(Eul[0]))
		

	#Checks which side of the head the drone is on (+) = right, (-) = left
	#This contextualizes the angle and locks it into the head reference frame
	LR = unit[0]*math.cos(Eul[0]-np.pi/2)+unit[1]*math.sin(Eul[0]-np.pi/2)

	#LRdot = math.acos(unit[0]*math.cos(Eul[0]-np.pi/2)+unit[1]*math.sin(Eul[0]-np.pi/2))


	if ang > 90: #resets for angles behind the user
		ang = 180-ang

	#convert back to radians
	ang = ang/(180/np.pi)

	#using sin/cos to scale volume between L/R
	#this ensure conitnuity when switching between sides
	if LR > 0: #on the right
		right = vol*math.sin(ang/2 + np.pi/4)
		left = vol*math.cos(ang/2 + np.pi/4)

	elif LR < 0: #on the left
		left = vol*math.sin(ang/2 + np.pi/4)
		right = vol*math.cos(ang/2 + np.pi/4)

	#Prints the L/R/Vol commands to the terminal
	os.system("amixer sset Master {}%,{}%".format(left, right))
	print(left, right, ang)
	

def listener():
    #creates a node for listening
    rospy.init_node('listener', anonymous=True)

    #subscribes to the two vicon objects
    par = message_filters.Subscriber('/quad_pose', TransformStamped)
    will = message_filters.Subscriber('/head_pose', TransformStamped)

    #passes the data into the main callback function
    ts = message_filters.TimeSynchronizer([par, will], 10)
    ts.registerCallback(callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

#main function that triggers everything
while 1==1:
	listener()


