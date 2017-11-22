#!/usr/bin/env python
import numpy as np

aveV 		= 55 #average velocity
accA 		= 10 #acceleration
decA 		= -10#deceleration 
maxSpeed 	= 70 #maxspeed of the car
clearance 	= 4.0 # This a magic value don't question why ? Dont Set it below 4
gamma		= 0.9 # Discount Factor



################################dynamics###############################
deltaT = 0.1 #each time step represent deltaT second
def nextXV(x, v, a, maxSpeed):
    vNext = v + a*deltaT
    if(vNext < 0):#front car never go back
        vNext = 0
    elif(vNext > maxSpeed):
        vNext = maxSpeed
    xNext = x + vNext*deltaT
    return [xNext, vNext]

################################reward###############################
def reward(s, target):###need to think about the size of car later
    if s <=0:
        return [-10**10, True]
    else:
        return [-abs(s-target), False]

################################reward###############################
def reward2(s, target, t):###need to think about the size of car later
    if s <=clearance:
        return -10**10 + t*1000
    else:
        return -abs(s-clearance-target)

################################Estimate xf and vf###############################
def estXV(xf_now, xf_prev):
    v = (xf_now - xf_prev)/deltaT
    x = xf_now + v*deltaT
    return x, v

################################discretize###############################
def disS(dis, vdiff, va, maxdis=120, minvdiff=-17, maxvdiff=17, maxva=17):
    #states: distance, relative velocity, agent velocity
    #dis:0 - 99  #100 states
    #vdiff: - 17 - 17 -> 0 - 34 #36 states
    #va:0 - 17 #18 states
    va = np.floor(va)
    vdiff = np.floor(vdiff)
    dis = np.floor(dis)    
    if(va >= maxva):
        va = maxva
    if(va < 0):
        va = 0
        print("error: velocity is negative")
    if(vdiff >= maxvdiff):
        vdiff = maxvdiff
    if(vdiff <= minvdiff):
        vdiff = minvdiff
    if(dis >= maxdis):
        dis = maxdis
    if(dis <= 0):
        dis = 0
    vdiff = vdiff - minvdiff
    return dis, vdiff, va
    
################################Value###############################
#value function will be a vector
#this function returns the index for value function of corresponding states
def Vindex(ddis, dvdiff, dva, maxdis= 120, minvdiff=-17, maxvdiff=17, maxva=17): #input must be discretized before hand
    nvdiff = maxvdiff- minvdiff + 1 # number of states of vdiff
    nva = maxva + 1 #number of states of va
    return int(ddis*nvdiff*nva + dvdiff*nva + dva)
