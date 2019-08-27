#!/usr/bin/env python

import rospy
import os
import subprocess
import time
import math
import message_filters
import numpy as np
import transforms3d
from geometry_msgs.msg import TransformStamped


class Server:
	def __init__(self):
		self.quad_pose = None
		self.oculus = None

	def quad_callback(self, msg):
		self.quad_pose = msg
		#print("quad")

		self. compute_stuff()

	def oculus_callback(self, msg):
		self.oculus = msg
		#print("oculus")
		
		self.compute_stuff()

	def compute_stuff(self):
		if self.quad_pose is not None and self.oculus is not None:
			pass
			#take in the positions
			p = self.quad_pose.transform.translation
			w = self.oculus.transform.translation
			

			#distances between subject/drone
			x_d = p.x - w.x
			y_d = p.y - w.y
			z_d = p.z - w.z


			#Spatial vector (3D)
			dist = math.sqrt(x_d**2 + y_d**2 + z_d**2)
			print(dist)
			

			#adjust the volume to decrease as 1/(1+R) from subject
			#Done so that at R = 0, vol = 100%
			vol = 100/(1+dist)
			#print(vol)

			#Prints the L/R/Vol commands to the terminal
			subprocess.call(["amixer", "sset", "Master", "{}%".format(vol), "{}%".format(vol)])

			#print(vol)
			

if __name__== '__main__':
    #creates a node for listening
    rospy.init_node('listener', anonymous=True)

    server = Server()

    #subscribes to the two vicon objects
    rospy.Subscriber('/quad_pose', TransformStamped, server.quad_callback)
    rospy.Subscriber('/head_pose', TransformStamped, server.oculus_callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()



