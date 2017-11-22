#!/usr/bin/env python

import  os  
import  sys  
import  tty, termios  
import roslib; roslib.load_manifest('arka_ryu_project')  
import rospy 
import pygame 
from pygame.locals import * 
from std_msgs.msg import String  
from arka_ryu_project.msg import *
from geometry_msgs.msg import Twist
from nav_msgs.msg import *
from math import *
import tf
import numpy as np
import json
import front_car_model2 as fc
from common_utils2 import *

class FrontCar:
	def __init__(self):
		self.start_game 	= False
		self.hz			= rospy.get_param('~hz',10)
		self.rate		= rospy.Rate(self.hz)
		self.pub		= rospy.Publisher('robot_1/cmd_vel',Twist,queue_size=1)
		self.time		= 0	# A counter to index the dynamics array.
		self.x_l, self.v_l	= fc.gen_dynamics2(100,1488,7,aveV, accA, decA, maxSpeed)  
	
	def gameStarts(self, msg):
		if msg.state == 1 or msg.state == 2:
			self.start_game = True
		
			# Precomute The Dynamics of Front Car



	def run(self):
		while not rospy.is_shutdown():
			if self.start_game:
				if(self.time >= len(self.v_l)):
					vel = 0
				else:
					vel = self.v_l[self.time] # Fill it up
				yaw = 0
				self.send_control(self.pub,vel,yaw)
				self.time = self.time + 1

			self.rate.sleep()

	def send_control(self,pub,vel,yaw):
		msg 		= Twist()
		msg.linear.x 	= vel
		msg.angular.z	= 0
		pub.publish(msg)

if __name__=="__main__":
	rospy.init_node('front_car2')
	front_car = FrontCar()
	rospy.Subscriber('robot_0/highway_game_start',RecordState,front_car.gameStarts,queue_size=1)
	front_car.run()
	#for elem in front_car.x_l:
	#	print elem
