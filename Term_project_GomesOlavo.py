#!/usr/bin/env python

import multiprocessing as mp
import random
import string
import common
import ach
import sys
from ctypes import *
import socket
import cv2.cv as cv
import cv2
import numpy as np
import math
import csv
import serial
from time import time, sleep
from math import pi
from functions_IK import *
from dynamixel_position import *

# CV setup 
cv.NamedWindow("wctrl_1", cv.CV_WINDOW_AUTOSIZE)
cv.NamedWindow("wctrl_2", cv.CV_WINDOW_AUTOSIZE)
newx = 320
newy = 240
nx = 320
ny = 240

N = 5

chan_1 = ach.Channel(common.CHAN_1)
chan_1.flush()

chan_2 = ach.Channel(common.CHAN_2)
chan_2.flush()

chan_3 = ach.Channel(common.CHAN_3)
chan_3.flush()

chan_4 = ach.Channel(common.CHAN_4)
chan_4.flush()

chan_5 = ach.Channel(common.CHAN_5)
chan_5.flush()

chan_6 = ach.Channel(common.CHAN_6)
chan_6.flush()

chan_7 = ach.Channel(common.CHAN_7)
chan_7.flush()

v = ach.Channel(common.ROBOT_CHAN_VIEW)
v.flush()

r = ach.Channel(common.MOTOR_CHAN)
r.flush()

#################################### PROCESS 1 ########################################################

# Define process to communicate with the simulated robot
def process_1():
	print 'Process 1: Running.....'

	while True:

	    # Send position received to the motors
	    motor = common.H_REF()
	    [status, framesize] = chan_6.get(motor, wait=False, last=True)
	    if status == ach.ACH_OK or status == ach.ACH_MISSED_FRAME or status == ach.ACH_STALE_FRAMES:
		print motor.ref[0],motor.ref[1]
        	r.put(motor)
    	    else:
        	raise ach.AchException(v.result_string(status))

	    # Get Frame from camera
	    img = np.zeros((newx,newy,3), np.uint8)
	    c_image = img.copy()
	    vid = cv2.resize(c_image,(newx,newy))
	    [status, framesize] = v.get(vid, wait=True, last=True)
	    if status == ach.ACH_OK or status == ach.ACH_MISSED_FRAME or status == ach.ACH_STALE_FRAMES:
		vid2 = cv2.resize(vid,(nx,ny))
		img = cv2.cvtColor(vid2,cv2.COLOR_BGR2RGB)
	    else:
		raise ach.AchException( v.result_string(status) )

	    # Send Image to the Open CV
	    chan_3.put(img)

#################################### PROCESS 2 2########################################################
           		
# Define process to communicate with the real robot
def process_2():
	print 'Process 2: Running.....'

	capture = cv2.VideoCapture(2)
	dynamixel = serial.Serial('/dev/ttyUSB0', baudrate=1000000)

	high_angle = 120.0 * (math.pi/180.0)
	low_angle = -120.0 * (math.pi/180.0)
	
	while True:

	    # Send position received to the motors
	    motor = common.H_REF()
	    [status, framesize] = chan_6.get(motor, wait=False, last=True)
	    if status == ach.ACH_OK or status == ach.ACH_MISSED_FRAME or status == ach.ACH_STALE_FRAMES:
		#print motor.ref[0],motor.ref[1]
		print "Got position"
		motor.ref[0] = min(max(-motor.ref[0], low_angle), high_angle)
		motor.ref[1] = min(max(-motor.ref[1], low_angle), high_angle)
		
		set_position(dynamixel, 3, motor.ref[0])
		set_position(dynamixel, 2, motor.ref[1])
        	#r.put(motor)
    	    else:
        	raise ach.AchException(v.result_string(status))

	    # Get Frame from camera
	    ret, frame = capture.read()
	    frame = cv2.resize(frame,(newx,newy))
	    #frame = cv2.cvtColor(frame, cv.CV_BGR2HSV)
	    frame = cv2.cvtColor(frame,cv2.COLOR_BGR2RGB)

	    # Send Image to the Open CV
	    chan_3.put(frame)
	

#################################### PROCESS 3 ########################################################

# Define process to process the image CV
def process_3():
	print 'Process 3: Running.....'

	# Getting information from UI
	ref = common.UI_REF()
	[status, framesize] = chan_7.get(ref, wait=True, last=True)
    	if status == ach.ACH_OK or status == ach.ACH_MISSED_FRAME or status == ach.ACH_STALE_FRAMES:
        	pass
    	else:
        	raise ach.AchException(v.result_string(status))

	# Looking which color we need to track
	if (ref.color[0] == 1.0):
		low_red = 0.0
		low_green = 0.0
		low_blue = 50.0
		high_red = 50.0
		high_green = 50.0
		high_blue = 255.0
	elif (ref.color[0] == 2.0):
		low_red = 0.0
		low_green = 50.0
		low_blue = 0.0
		high_red = 50.0
		high_green = 255.0
		high_blue = 50.0
	elif (ref.color[0] == 3.0):
		low_red = 50.0
		low_green = 0.0
		low_blue = 0.0
		high_red = 255.0
		high_green = 50.0
		high_blue = 50.0

	# Getting the center of the ball on the image
	center = common.CV_REF()
	while True:
		img_cv = np.zeros((newx,newy,3), np.uint8)
	    	c_image_cv  = img_cv .copy()
	    	vid_cv  = cv2.resize(c_image_cv ,(newx,newy))
		[status, framesize] = chan_3.get(vid_cv , wait=True, last=True)
    		if status == ach.ACH_OK or status == ach.ACH_MISSED_FRAME or status == ach.ACH_STALE_FRAMES:
    			frame = cv2.inRange(vid_cv , np.array((low_blue,low_green,low_red)), np.array((high_blue,high_green,high_red)))
    			[ys, xs] = frame.nonzero()
			if (len(ys) > 0):
				center.ref[2] = 1.0 # ON screen
				val_x = xs.mean()
				val_y = ys.mean()			
				x = val_x / (nx / 2.0) - 1.0
				y = -val_y / (ny / 2.0) + 1.0
				#print x,y
				center.ref[0] = x
				center.ref[1] = y
				
				cv2.rectangle(frame, (int(val_x)-2, int(val_y)-2), (int(val_x)+3, int(val_y)+3), 0x7F, cv.CV_FILLED)
				cv2.rectangle(vid_cv,(int(val_x)-2, int(val_y)-2), (int(val_x)+3, int(val_y)+3),(0,255,0),2)
	      			text = "x = %3.2f, y = %3.2f" % (x,y)
	      			cv2.putText(vid_cv, text, (20,30),cv2.FONT_HERSHEY_SIMPLEX,0.7,(0,0,255),2)

			else:
				center.ref[2] = 0.0 # OFF screen

			
			cv2.imshow("wctrl_2", frame)
			cv2.waitKey(10)
			cv2.imshow("wctrl_1", vid_cv)
			cv2.waitKey(10)

			chan_4.put(center)	
    		else:
        		raise ach.AchException(v.result_string(status))
	
#################################### PROCESS 4 ########################################################

def PID(err,err_0,Kp,Kd,Ki,period):
	theta_rp = err*Kp
	theta_rpd = theta_rp + Kd*(err-err_0)/period
	err_i1 = err*period
	theta_rpid = theta_rpd + Ki*(err_0 + err_i1)
	return theta_rpid

def vel2pos(pos_0,velos,period):
	return pos_0 + velos*period

# Define process to do the PID
def process_4():
	print 'Process 4: Running.....'

	# Getting information from UI
	ref = common.UI_REF()
	[status, framesize] = chan_7.get(ref, wait=True, last=True)
    	if status == ach.ACH_OK or status == ach.ACH_MISSED_FRAME or status == ach.ACH_STALE_FRAMES:
        	pass
    	else:
        	raise ach.AchException(v.result_string(status))

	# Calculating the command to send to the IK
	Kp = ref.gains[0]
	Kd = ref.gains[1]
	Ki = ref.gains[2]

	freq = 50.0
	period = 1/freq
	err_0 = [0.0, 0.0]
	err_i0 = [err_0[0]*period, err_0[1]*period]
	pos_0 = [0.0, 0.0]

	i = 0

	center = common.CV_REF()
	desire = common.H_REF()
	while(True):
		i += 1
		startTime = time() 
		[status, framesize] = chan_4.get(center, wait=True, last=True)
		if status != ach.ACH_OK and status != ach.ACH_STALE_FRAMES and status != ach.ACH_MISSED_FRAME:
			raise ach.AchException(Input.result_string())
	
		err = [ref.ref[0]-center.ref[0], ref.ref[1]-center.ref[1]]
		#print err

		if(center.ref[2] == 1.0):
			pos_x = vel2pos(pos_0[0],PID(err[0],err_0[0],Kp,Kd,Ki,period),period)
			pos_y = vel2pos(pos_0[1],PID(err[1],err_0[1],Kp,Kd,Ki,period),period)
			pos_0 = [pos_x, pos_y]
		else:
			pos_x = vel2pos(pos_0[0],-0.5,period) if abs((i % 320) - 160) < 80 else vel2pos(pos_0[0],0.5,period)
			pos_y = 0 #vel2pos(0.5,1) if abs((i % 320) - 160) < 80 else vel2pos(-0.5,1)
			pos_0 = [pos_x, pos_y]
	
		err_0 = err
		desire.ref[0] = pos_0[0]
		desire.ref[1] = pos_0[1]
		chan_5.put(desire)
	
		timeTaken = startTime + period - time()
		if (timeTaken > 0):
			sleep(timeTaken)

#################################### PROCESS 5 ########################################################


# Define process to do the IK - Jacobian
def process_5():
	print 'Process 5: Running.....'
	desire = common.H_REF()
	motor = common.H_REF()

	while True:
		[status, framesize] = chan_5.get(desire, wait=True, last=True)
	    	if status == ach.ACH_OK or status == ach.ACH_MISSED_FRAME or status == ach.ACH_STALE_FRAMES:
			theta = numpy.array([0.0,0.0])   	# Initial Angle
			err = 1	  			# Maximum Error
			dd_theta = 0.01 			# Delta theta
			stp = 0.01	  			# Step
			e = get_FK(theta)
			g = numpy.array([desire.ref[0],desire.ref[1]])
			count = 0
			chan_6.put(desire)
			#print g
			while(get_dist(e,g) > err):
				J = get_J(dd_theta, theta)
				#print J
				Jp = numpy.linalg.pinv(J)
				#print Jp
				de = get_next_point_delta(e,g,stp)
				#print de
				d_theta = numpy.dot(Jp,numpy.transpose(de))
				#print d_theta
				theta = theta + numpy.transpose(d_theta)
				#print theta
				#theta = wrapTo2Pi(theta)
			    	e = get_FK(theta)
				#print e

				motor.ref[0] = e[0]
				motor.ref[1] = e[1]
				chan_6.put(motor)

				count = count + 1
				#print count
			    	if (count > 20000):
					break
			
	    	else:
			raise ach.AchException(v.result_string(status))



##################################### MAIN ########################################################

print "\n----- TERM PROJECT : Jose Lucas Gomes Olavo -----\n"
print "Please enter the parameters:\n"
print "1 - Desired horizontal position of the ball on the camera (normalized [-1 1])\n"
print "2 - Desired vertical position of the ball on the camera (normalized [-1 1])\n"
print "3 - Kp: Proportional Gain\n"
print "4 - Kd: Derivative Gain\n"
print "5 - Ki: Integral Gain\n"
print "6 - Color of the ball to track: B = Blue | G = Green | R = Red\n"
print "7 - Flag: 0 = Simulated Robot | 1 = Real Robot\n"

x_d = float(raw_input("Enter parameter 1: "))
y_d = float(raw_input("Enter parameter 2: "))
Kp = float(raw_input("Enter parameter 3: "))
Kd = float(raw_input("Enter parameter 4: "))
Ki = float(raw_input("Enter parameter 5: "))
Color = raw_input("Enter parameter 6: ")
Flag = float(raw_input("Enter parameter 7: "))

print "\nEnd of selection: THANK YOU!\n"
print "Values: x_d=%.1f y_d=%.1f Kp=%.1f Kd=%.1f Ki=%.1f Color=%c Flag=%.1f\n" % (x_d,y_d,Kp,Kd,Ki,Color,Flag)

#x_d = 0.0
#y_d = 0.0
#Kp = 0.8
#Kd = 0.6
#Ki = 0.2
#Color = "B"
#Flag = 0.0

param = common.UI_REF()
param.ref[0] = x_d
param.ref[1] = y_d
param.gains[0] = Kp
param.gains[1] = Kd
param.gains[2] = Ki
if (Color == "B"):
	param.color[0] = 1.0
if (Color == "G"):
	param.color[0] = 2.0
if (Color == "R"):
	param.color[0] = 3.0
param.flag[0] = Flag


# Process that are running
if (Flag == 0.0):
	processes = [mp.Process(target=process_1), mp.Process(target=process_3), mp.Process(target=process_4),mp.Process(target=process_5)]
else:
	processes = [mp.Process(target=process_2), mp.Process(target=process_3), mp.Process(target=process_4),mp.Process(target=process_5)]


# Run processes
for p in processes:
    p.start()
    #print p


chan_7.put(param)


try:
	while True:
		pass

except KeyboardInterrupt:
	# Exit the completed processes
	for p in processes:
    		p.terminate()

	motor_zero = common.H_REF()
	motor_zero.ref[0] = 0.0
	motor_zero.ref[1] = 0.0
	r.put(motor_zero)

	# Close the connection to the channels
	chan_1.close()
	chan_2.close()
	chan_3.close()
	chan_4.close()
	chan_5.close()
	chan_6.close()
	chan_7.close()
	v.close()
	r.close()
	#dynamixel.close()


