#!/usr/bin/env python

# Jose Lucas Gomes Olavo 

import math
import numpy
from copy import deepcopy

l1 = 0.0
l2 = 0.0

class stru:
    def __init__(self):
        self.x = 0
        self.y = 0
	self.z = 0
	self.thetax = 0
	self.thetay = 0
	self.thetaz = 0

# Motor 1
Motor_1 = stru()
Motor_1.z = l1
Motor_1.thetaz = 1

# Motor 2
Motor_2 = stru()
Motor_2.z = l2
Motor_2.thetay = 1


# Forward kinematics
def get_FK(test):
	rot_1 = test[0]
	rot_2 = test[1]
	return numpy.array([rot_1,rot_2])

def get_Pos(T):	
	x = T[0,3]
	y = T[1,3]
	z = T[2,3]
	thetax = math.atan2(T[2,1],T[2,2])
	thetay = math.atan2(-T[2,0],(T[2,1]**2 + T[2,2]**2))
	thetaz = math.atan2(T[1,0],T[0,0])
	#print [x,y,z,thetax,thetay,thetaz]	
	return numpy.array([thetaz,thetay])


# Euclidian Distance
def get_dist(e,g):
	tam_e = len(e)
	tam_g = len(g)
	summ = 0
	if (tam_e == tam_g):
		for i in range (0,tam_e):
			summ = (e[i] - g[i])**2 + summ
		d = math.sqrt(summ)
	return d

# Jacobian
def get_J(d_theta, theta):
	e = get_FK(theta)
	J = numpy.zeros(shape=(len(e),len(theta)))
	for i in range (0,len(e)):
		for j in range (0,len(theta)):
			theta_new = deepcopy(theta)
			theta_new[j] = theta_new[j] + d_theta
			de = get_FK(theta_new) - e
			J[i,j] = de[i]/d_theta
	return J

# Get next Point
def get_next_point_delta(e,g,stp):
	m = g - e
	#print m
	m = m/numpy.linalg.norm(m)
	#print m
	p = m*stp
	#print p
	return p

def wrapTo2Pi(ang):
    return ang % (2*math.pi)


