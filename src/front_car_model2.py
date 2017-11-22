#!/usr/bin/python
import numpy as np
import matplotlib.pyplot as plt
from common_utils2 import *

#the behavior of front car for T time horizon
#def gen_dynamics1(T):#T is the time-horizon. We assume each time step is 0.1 s
#    vf = np.zeros(T+1)#velocity
#    xf = np.zeros(T+1)#location
#    xf[0] = np.random.normal(20,5)#initial location
#    vf[0] = np.random.normal(40,1)#initial velocity
#    stop = False #stop flag. If true, the front car start decelerate
#    accelerate = False #accelerate flag. If true, the car accelerate
#    for t in range(T):
#        print(t)
#        if(stop): # if The Stop flag is raised you deterministically choose to decelerate
#           af = np.random.normal(-40,2)
#           xf[t+1],vf[t+1] = nextXV(xf[t],vf[t],af)
#           if(vf[t] == 0):#if the velocity is zero, start accelerating again
#               stop = False
#               accelerate = True    
#        else:
#            u = np.random.uniform(0,1)
#            if(u < 0.99):#the car will drive normally w.p. 0.99. Otherwise, it starts decelerating
#                if(accelerate):#if the car stops, give it large acceleration
#                    af = np.random.normal(40,2)
#                    xf[t+1],vf[t+1] = nextXV(xf[t],vf[t],af)
#                    if(vf[t+1] > 40):#stop accelerating if the next velocity exceeds 40
#                        accelerate = False
#                else: #stable driving
#                    af = np.random.normal(0,1)
#                    xf[t+1],vf[t+1] = nextXV(xf[t],vf[t],af)
#            else:#decelerate
#                stop = True
#                af = np.random.normal(-40,2)
#                xf[t+1],vf[t+1] = nextXV(xf[t],vf[t],af)
#                if(vf[t] == 0):
#                    stop = False
#    return [xf,vf]

# The behaviour of the front car
def gen_dynamics2(init_x, goal_x, seed, aveV, accA, decA, maxSpeed):
	np.random.seed(seed)

	if(goal_x <= init_x):
		raise IOError('goal pos must be greater the init pose')

	# Computed States as time series
	vf = []
	xf = []
	
	# Initial Velocities and position
	x = init_x
	v = np.random.normal(aveV,1)
	t = 0
	xf.append(x)
	vf.append(v)


	# Some Flags to indicate acceleration/deceleration	
	stop 		= False #stop flag. If true, the front car start decelerate
	accelerate 	= False #accelerate flag. If true, the car accelerate

	# Next States
	next_x		= 0
	next_v		= 0
	count 		= 0		
	while (x < goal_x):
		#print(x)
 
		# if The Stop flag is raised you deterministically choose to decelerate
		if(stop):
	       		af = np.random.normal(decA,2)
	       		next_x, next_v = nextXV(x,v,af,maxSpeed)

			#if the velocity is zero, start accelerating again
			if(next_v <= 0):
				#count += 1
				#if (count == 15):
				stop 		= False
				accelerate 	= True
					#count = 0

		else:
	        	u = np.random.uniform(0,1)

			#the car will drive normally w.p. 0.59. Otherwise, it starts decelerating
	        	if(u < 0.99):
				if(accelerate):#if the car stops, give it large acceleration
					af = np.random.normal(accA,2)
	       				next_x, next_v = nextXV(x,v,af,maxSpeed)
	                	
					if(next_v > aveV):#stop accelerating if the next velocity exceeds 40
						accelerate = False
	            	
				else: #stable driving
	                		af = np.random.normal(0,1)
	                		next_x, next_v = nextXV(x,v,af,maxSpeed)
	        
			else:#decelerate
				stop = True
				af = np.random.normal(decA,2)
				next_x, next_v = nextXV(x,v,af,maxSpeed)

				if(next_v <= 0):
					stop = False

		x = next_x
		v = next_v
		xf.append(x)
		vf.append(v)

	return (xf,vf)

if __name__=="__main__":
	x, v = gen_dynamics2(0,180)
	#print v
