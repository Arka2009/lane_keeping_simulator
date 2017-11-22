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
from common_utils import *

class AgentCar:
	def __init__(self):
		self.start_game 	= False
		self.hz			= rospy.get_param('~hz',10)
		self.rate		= rospy.Rate(self.hz)
		self.pub		= rospy.Publisher('robot_0/cmd_vel',Twist,queue_size=1)
		self.time		= 0
		
		# Evalute (Offline) the front car dynamics
		self.xf, self.vf	= fc.gen_dynamics2(40,1488,1,aveV, accA, decA, maxSpeed)
		self.T			= len(self.xf)

		self.xa, self.va	= self.gen_dynamics()

		# Read The pose of the agent car and the front car
		self.pose_xf		= 0
		self.pose_xa		= 0

		# Collided
		self.collided		= False

	def findAction(self,x, v, actionSet, t, xf,vf):

        	Values = np.zeros(len(actionSet))
        	count = 0
        	#estimate front car
        	if(t!=0):
            		nextxf, nextvf = estXV(xf[t], xf[t-1])
        	else:
            		nextxf, nextvf = [xf[0] + deltaT*vf[0], vf[0]]
        	for a in actionSet:
            		nextx, nextv = nextXV(x, v, a, maxSpeed)#next x and v of agent
            		nexts = np.array([nextxf-nextx, nextvf-nextv, nextv])
            		nexts = disS(nexts[0],nexts[1],nexts[2])
            		sInd = Vindex(nexts[0],nexts[1],nexts[2])
            		target = np.max([nextv*2+clearance, 10+clearance])
            		r = reward2(nextxf-nextx, target, t)
            		Values[count] = r  #+ gamma*disV[sInd]
            		count += 1
        	ind = np.argmax(Values)
        	return actionSet[ind]

	def gen_dynamics(self):
		T, xf, vf = (self.T, self.xf, self.vf)
    		#location and velocity of agent
    		xa = np.zeros(T)
		va = np.zeros(T)
		xa[0] = 10 #initial value for the agent
		va[0] = 0 # initial velocity for the agent
		#preparation
		hit = False
		states, actions, rewards, obs = [],[],[],[]
		total_reward = 0
		actionSet = range(-10,10,2) 
		#observation
		xr = xf[0] - xa[0] #relative location (distance)   
		vr = vf[0] - xa[0] #relative velocity  
		obs.append([xr, vr, va[0]]) #observation
		#states
		s0, s1, s2 = disS(xr, vr, va[0]) #S is discretized
		s = [s0, s1, s2]
		states.append(s)
		#reward
		target = np.max([s2*2+clearance, 10+clearance])
		r = reward2(s0, target, 0)
		rewards.append(r)
		
		
		for t in range(T-1): 
			a = self.findAction(xa[t],va[t],actionSet, t, xf,vf)
			#observation
			xa[t+1], va[t+1] = nextXV(xa[t], va[t], a, maxSpeed) # next x and v of agent 
			if(t!=0):#estimate next x and v of front car  
		    		nextxf, nextvf = estXV(xf[t], xf[t-1]) 
			else:
		    		nextxf, nextvf = [xf[0] + deltaT*vf[0], vf[0]]
			xr = nextxf - xa[t+1] #relative location (distance)   
			vr = nextvf - va[t+1] #relative velocity 
			obs.append([xr,vr,va[t+1]])
			#States
			s0, s1, s2 = disS(xr, vr, va[t+1])
			nexts = [s0, s1, s2]
			#reward
			target = np.max([s2*2+clearance,10+clearance])
			if(xf[t+1] - xa[t+1] <= clearance):#hit the car
		 		r = -10**10
				hit = True
			else:
		    		r = reward2(s0, target, t)
			s = nexts
			states.append(s)
			actions.append(a)
			rewards.append(r)
			total_reward += r     
			if hit: 
		    		print("our agent hit the front car")
		    		#print(total_reward)
		    		return xa,va
		    	#break
			#print(total_reward)
		return xa,va

	# Extract the agent car pose
	def extract_pose_xa(self,msg):
		seq			= msg.header.seq
		self.pose_xa 		= msg.pose.pose.position.x
		self.vel_xa 		= msg.twist.twist.linear.x
	
	# Extract the front car pose
	def extract_pose_xf(self,msg):
		seq			= msg.header.seq
		self.pose_xf 		= msg.pose.pose.position.x
		self.vel_xf 		= msg.twist.twist.linear.x

	# Collision Checker
	def collision_check(self):
		xa = self.pose_xa
		xf = self.pose_xf

		if(not self.collided):
			if (abs(xa-xf) <= clearance):
				self.collided = True
			else:
				self.collided = False
		else:
			self.collided = True
		#if(self.vel_xa == 0):
		#	print("clearance = "+str(abs(xa-xf)))
		#return False
	
	def gameStarts(self, msg):
		if msg.state == 1 or msg.state == 2:
			self.start_game = True

	def run(self):
		while not rospy.is_shutdown():
			if self.start_game:
				if(self.time >= len(self.va)):
					vel = 0
				else:
					vel = self.va[self.time] # Fill it up
				yaw = 0
				self.send_control(self.pub,vel,yaw)
				self.time = self.time + 1
			self.rate.sleep()

	def send_control(self,pub,vel,yaw):
		msg 		= Twist()
		self.collision_check()
		if(self.collided):
			msg.linear.x 	= 0
		else:
			msg.linear.x 	= vel
		msg.angular.z	= 0
		pub.publish(msg)

if __name__=="__main__":
	rospy.init_node('agent_car')
	agent_car = AgentCar()
	rospy.Subscriber('robot_0/highway_game_start',RecordState,agent_car.gameStarts)
	rospy.Subscriber('robot_1/base_pose_ground_truth',Odometry,agent_car.extract_pose_xf)
	rospy.Subscriber('robot_0/base_pose_ground_truth',Odometry,agent_car.extract_pose_xa)
	agent_car.run()
	#xa, va = agent_car.gen_dynamics()
	#for elem in xa:
	#	print elem
	#for elem in va:
	#	print elem
