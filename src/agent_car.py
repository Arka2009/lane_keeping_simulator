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
import front_car_model as fc

class AgentCar:
	def __init__(self):
		self.start_game 	= False
		self.hz			= rospy.get_param('~hz',10)
		self.rate		= rospy.Rate(self.hz)
		
		# Front Car Observation
		self.front_x		= 0
		self.front_v		= 0
		self.front_prev_x	= 0
		self.front_prev_v	= 0
		
	def gameStarts(self, msg):
		if msg.state == 1 or msg.state == 2:
			self.start_game = True

	def extract_front_car(self,msg):
		self.front_prev_x 	= self.front_x
		self.front_prev_v 	= self.front_v
		seq			= msg.header.seq
		self.front_x 		= msg.pose.pose.position.x
		self.front_v		= msg.twist.twist.linear.x
		
	def run(self):
		while not rospy.is_shutdown():
			if self.start_game:
			self.rate.sleep()


if __name__=="__main__":
	#rospy.init_node('agent_car')
	#agent_car = AgentCar()
	#rospy.Subscriber('robot_0/highway_game_start',RecordState,agent_car.gameStarts)
	#rospy.Subscriber('robot_1/base_pose_ground_truth',Odometry,agent_car.extract_front_car)
	#agent_car.run()
	
