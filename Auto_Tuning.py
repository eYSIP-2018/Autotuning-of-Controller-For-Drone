#!/usr/bin/env python
import rospy
import roslib
from plutodrone.srv import *
from plutodrone.msg import *
from pid_tune.msg import PidTune
from std_msgs.msg import Char,Int16,Float64,Empty
import time
import math
import sys, select, termios, tty
from geometry_msgs.msg import Twist,PoseArray

#Explanation of code:
#Initially Kp, Kd & Ki are set to minimum. First, the drone gets armed. Kp increases wrt rate of change of error.
#After the drone crosses a circle of radius 1.8 units ten times, Kp is set constant. Kd is increased again wrt rate
#of change of error till twice amplitude is less than 1 unit. The mentioned condition should be true for 10 cycles.
#Then Ki and Kd are increased till the drone maintains itself in a position of 0.8 units circle. Again this condition is
#checked ten times

x_co=0.0
y_co=0.0
z_co=0.0
kp_x=0.0
kp_y=0.0
kp_z=0.0
a=0
w=0
r=0
flag_initialize=0
global i
global j
global timeout1
global timeout
i=0
j=0
timeout=0
timeout1=0
count=0

err_x=[0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
err_y=[0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
err_z=[0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]

err_x_previous=[0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
err_y_previous=[0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
err_z_previous=[0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]

err_x_auto=0.0
err_y_auto=0.0
err_z_auto=0.0

err_prior_x_auto=0.0
err_prior_y_auto=0.0
err_prior_z_auto=0.0

err=[0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
err_prior=[0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
out=[0.0,0.0,0.0]
now_time=0.0
prev_time=0.0
prev_whycon=0.0
now_whycon=0.0
prev_ki=0.0
now_ki=0.0
start_time_tuning=0.0
Mp_max=0
Mp_min=0
flag_Kp_start=0
sumerr=[0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
sumerr1=[0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
max_vel=120
min_vel=-120
max_vel_z=250
min_vel_z=-250
Kp=[0.0,0.0,0.0]
Ki=[0.0,0.0,0.0]
Kd=[0.0,0.0,0.0]

Kp_max=[0,0,0]
Kp_min=[0,0,0]

Kd_max=[0,0,0]
Kd_min=[0,0,0]

Ki_max=[0,0,0]
Ki_min=[0,0,0]

kp_auto_x=0.0
kp_auto_y=0.0
kp_auto_z=0.0
kd_auto_x=0.0
kd_auto_y=0.0
kd_auto_z=0.0
ki_auto_x=0.0
ki_auto_y=0.0
ki_auto_z=0.0

# list_co=[[0,0,29],[0,0,29],[0,0,29],[0,0,29]]
list_co=[[0,0,15],[-6,-4,18],[-6,4,19],[0,0,26]]

def callback1(msg):
  global x_co
  global y_co
  global z_co
  x_co=msg.poses[0].position.x
  y_co=msg.poses[0].position.y
  z_co=msg.poses[0].position.z

def Initialisation():
	global Kp_min
	global Kp_max
	global Kd_min
	global Kd_max
	global Ki_min
	global Ki_max
	global kp_auto_x
	global kp_auto_y
	global kp_auto_z
	global kd_auto_x
	global kd_auto_y
	global kd_auto_z
	global ki_auto_x
	global ki_auto_y
	global ki_auto_z

	print 'Enter the minimum Kp value for pitch, roll and z axis respectively'
	Kp_min[0]=input()
	Kp_min[1]=input()
	Kp_min[2]=input()
	print Kp_min
	print 'Enter the maximum Kp value for pitch, roll and z axis respectively'
	Kp_max[0]=input()
	Kp_max[1]=input()
	Kp_max[2]=input()
	print Kp_max
	print 'Enter the minimum Kd value for pitch, roll and z axis respectively'
	Kd_min[0]=input()
	Kd_min[1]=input()
	Kd_min[2]=input()
	print Kd_min
	print 'Enter the maximum Kd value for pitch, roll and z axis respectively'
	Kd_max[0]=input()
	Kd_max[1]=input()
	Kd_max[2]=input()
	print Kd_max
	print 'Enter the minimum Ki value for pitch, roll and z axis respectively'
	Ki_min[0]=input()
	Ki_min[1]=input()
	Ki_min[2]=input()
	print Ki_min
	print 'Enter the maximum Ki value for pitch, roll and z axis respectively'
	Ki_max[0]=input()
	Ki_max[1]=input()
	Ki_max[2]=input()
	print Ki_max

	kp_auto_x=Kp_min[0]
	kp_auto_y=Kp_min[1]
	kp_auto_z=Kp_max[2]

	ki_auto_x=Ki_min[0]
	ki_auto_y=Ki_min[1]
	ki_auto_z=Ki_min[2]

	kd_auto_x=Kd_min[0]
	kd_auto_y=Kd_min[1]
	kd_auto_z=Kd_min[2]

def motion():
  global err
  global err_x
  global err_y
  global err_z
  global Kp
  global Kd
  global Ki
  global kp_auto_x
  global kp_auto_y
  global kp_auto_z
  global kd_auto_x
  global kd_auto_y
  global kd_auto_z
  global ki_auto_x
  global ki_auto_y
  global ki_auto_z
  global flag_initialize
  if(flag_initialize==0): # Initial values are set with the help of this flag.
    Kp=[kp_auto_x,kp_auto_y,kp_auto_z]
    Ki=[ki_auto_x,ki_auto_y,ki_auto_z]
    Kd=[kd_auto_x,kd_auto_y,kd_auto_z]
    err[a]=list_co[count][0]-x_co
    err[a+1]=list_co[count][1]-y_co
    err[a+2]=list_co[count][2]-z_co
    flag_initialize=1
    PID(err)
  else:
    err[a]=list_co[count][0]-x_co #The zeroth, third, sixth..fifteenth values of the array store pitch error
    err[a+1]=list_co[count][1]-y_co #Similarly first, fourth, seventh...sixteenth values of the array store roll error
    err[a+2]=list_co[count][2]-z_co

    err_x[w]=list_co[count][0]-x_co #Array of 54 elements to store pitch error. This is used to find max and min values  
    err_y[w]=list_co[count][1]-y_co #Array of 54 elements to store roll error. This is used to find max and min values
    err_z[w]=list_co[count][2]-z_co #Array of 54 elements to store throttle error. This is used to find max and min values
    PID(err)

def PID(err):
  global count
  global now_time
  global prev_time
  global present_time
  global previous_time
  global out
  global sumerr
  global sumerr1
  global a
  global r
  global w
  global i
  global j
  global flag_Kp_start
  global now_whycon
  global prev_whycon
  global start_time_tuning
  global now_ki
  global prev_ki
  global err_x_auto
  global err_y_auto
  global err_z_auto
  global err_prior_x_auto
  global err_prior_z_auto
  global err_prior_y_auto
  global err_x_previous
  global err_y_previous
  global timeout
  now_time=time.time()
  delta_time=now_time-prev_time
  if((delta_time)>0.023): #Sample time
   for f in xrange(a,a+3,1): #As mentioned above, this is for arrangement of the array
     sumerr[f]=sumerr[f]+Ki[f%3]*(err[f])*delta_time
     sumerr1[f]=(err[f]-err_prior[f])/delta_time
     out[f%3] = Kp[f%3] * err[f] + sumerr[f] + Kd[f%3] * sumerr1[f]
     err_prior[f]=err[f]
     prev_time=now_time
  now_whycon=time.time()
  delta_whycon=now_whycon-prev_whycon
  if(delta_whycon>0.030): #Rate of subscribing whycon co-ordinates
    if(flag_Kp_start==0): #For Finding value of Kp
      print 'Kp tuning initiated'
      if(math.fabs(err[a])<1.8 and math.fabs(err[a+1])>1.8 and math.fabs(err[a+2])>2): #When pitch co-ordiinate is within limits and roll is not
        r=0
        err_z_auto=err[a+2]
        Kp[1]=Kp[1]+0.01 #rate of change of error
        Kp[2]=Kp[2]-(err_z_auto-err_prior_z_auto)/(delta_whycon*100)
        err_prior_z_auto=err_z_auto
      elif(math.fabs(err[a])<1.8 and math.fabs(err[a+1])>1.8 and math.fabs(err[a+2])<2): #When pitch co-ordiinate is within limits and roll is not
        r=0
        Kp[1]=Kp[1]+0.1 #rate of change of error
      elif(math.fabs(err[a])>1.8 and math.fabs(err[a+1])<1.8 and math.fabs(err[a+2])>2):
        r=0
        err_z_auto=err[a+2]
        Kp[0]=Kp[0]+0.1
        Kp[2]=Kp[2]-(err_z_auto-err_prior_z_auto)/(delta_whycon*100)
        err_prior_z_auto=err_z_auto
      elif(math.fabs(err[a])>1.8 and math.fabs(err[a+1])<1.8 and math.fabs(err[a+2])<2):
        r=0
        Kp[0]=Kp[0]+0.1
      elif(math.fabs(err[a])>1.8 and math.fabs(err[a+1])>1.8 and math.fabs(err[a+2])>2):
        r=0
        err_x_auto=err[a]
        err_y_auto=err[a+1]
        err_z_auto=err[a+2]
        Kp[0]=Kp[0]-(math.fabs(err_x_auto)-math.fabs(err_prior_x_auto))/(delta_whycon*100)
        Kp[1]=Kp[1]-(math.fabs(err_y_auto)-math.fabs(err_prior_y_auto))/(delta_whycon*100)
        Kp[2]=Kp[2]-(err_z_auto-err_prior_z_auto)/(delta_whycon*100)
        err_prior_x_auto=err_x_auto
        err_prior_y_auto=err_y_auto
        err_prior_z_auto=err_z_auto
      elif(math.fabs(err[a])>1.8 and math.fabs(err[a+1])>1.8 and math.fabs(err[a+2])<2):
        r=0
        err_x_auto=err[a]
        err_y_auto=err[a+1]
        Kp[0]=Kp[0]-(math.fabs(err_x_auto)-math.fabs(err_prior_x_auto))/(delta_whycon*100)
        Kp[1]=Kp[1]-(math.fabs(err_y_auto)-math.fabs(err_prior_y_auto))/(delta_whycon*100)
        err_prior_x_auto=err_x_auto
        err_prior_y_auto=err_y_auto
      elif(math.fabs(err[a])<=1.8 and math.fabs(err[a+1])<=1.8 and math.fabs(err[a+2])>2):
        r=0
        err_z_auto=err[a+2]
        Kp[2]=Kp[2]-(err_z_auto-err_prior_z_auto)/(delta_whycon*100)
        err_prior_z_auto=err_z_auto

      elif(math.fabs(err[a])<=1.8 and math.fabs(err[a+1])<=1.8 and math.fabs(err[a+2])<=1.8):
        r=r+1
        if(r==10): # To check the condition 10 times
          flag_Kp_start=1
          r=0
      print Kp
      print Kd
    elif(flag_Kp_start==1): #Kd tuning initiated
	  print 'Kp tuned'
	  print 'Kd tuning initiated'
	  if((math.fabs(max(err_x))-math.fabs(min(err_x)))<=1 and (math.fabs(max(err_y))-math.fabs(min(err_y)))<=1 and (math.fabs(max(err_z))-math.fabs(min(err_z)))<=1.6):
	#Twice the amplitude is less than 1 unit.
	    r=r+1
	    if(r==50): #Condition checked 50 times
	      r=0
	      flag_Kp_start=2
	#When pitch and roll are damped but altitude is not
	  elif((math.fabs(max(err_x))-math.fabs(min(err_x)))<=1 and (math.fabs(max(err_y))-math.fabs(min(err_y)))<=1 and (math.fabs(max(err_z))-math.fabs(min(err_z)))>1.6):
	    err_z_auto=err[a+2]
	    Kd[2]=Kd[2]+(math.fabs((err_z_auto)-(err_prior_z_auto))/(delta_whycon*150))
	    err_prior_z_auto=err_z_auto
	#When pitch and altitude are damped but roll is not  
	  elif((math.fabs(max(err_x))-math.fabs(min(err_x)))<1 and (math.fabs(max(err_y))-math.fabs(min(err_y)))>1 and (math.fabs(max(err_z))-math.fabs(min(err_z)))<1.6):
	    r=0
	    err_y_auto=err[a+1]
	    Kd[1]=Kd[1]+(math.fabs(err_y_auto-err_prior_y_auto)/(delta_whycon*150))
	    err_prior_y_auto=err_y_auto
	#When pitch is damped but roll and altitude are not
	  elif((math.fabs(max(err_x))-math.fabs(min(err_x)))<1 and (math.fabs(max(err_y))-math.fabs(min(err_y)))>1 and (math.fabs(max(err_z))-math.fabs(min(err_z)))>1.6):
	    r=0
	    err_z_auto=err[a+2]
	    err_y_auto=err[a+1]
	    Kd[2]=Kd[2]+(math.fabs((err_z_auto-err_prior_z_auto))/(delta_whycon*150))
	    Kd[1]=Kd[1]+(math.fabs(err_y_auto-err_prior_y_auto)/(delta_whycon*150))
	    err_prior_z_auto=err_z_auto
	    err_prior_y_auto=err_y_auto
	#When roll and altitude are damped but pitch is not    
	  elif((math.fabs(max(err_x))-math.fabs(min(err_x)))>1 and (math.fabs(max(err_y))-math.fabs(min(err_y)))<1 and (math.fabs(max(err_z))-math.fabs(min(err_z)))<1.6):
	    r=0
	    err_x_auto=err[a]
	    Kd[0]=Kd[0]+(math.fabs(err_x_auto-err_prior_x_auto)/(delta_whycon*150))
	    err_prior_x_auto=err_x_auto
	#When roll is damped but pitch and altitude are not 
	  elif((math.fabs(max(err_x))-math.fabs(min(err_x)))>1 and (math.fabs(max(err_y))-math.fabs(min(err_y)))<1 and (math.fabs(max(err_z))-math.fabs(min(err_z)))>1.6):
	    r=0
	    err_x_auto=err[a]
	    err_z_auto=err[a+2]
	    Kd[0]=Kd[0]+(math.fabs(err_x_auto-err_prior_x_auto)/(delta_whycon*150))
	    Kd[2]=Kd[2]+(math.fabs((err_z_auto-err_prior_z_auto))/(delta_whycon*150))
	    err_prior_z_auto=err_z_auto
	    err_prior_x_auto=err_x_auto
	#When pitch and roll are damped but altitude is not
	  elif((math.fabs(max(err_x))-math.fabs(min(err_x)))>1 and (math.fabs(max(err_y))-math.fabs(min(err_y)))>1 and (math.fabs(max(err_z))-math.fabs(min(err_z)))<1.6):
	    r=0
	    err_x_auto=err[a]
	    err_y_auto=err[a+1]
	    Kd[0]=Kd[0]+(math.fabs(err_x_auto-err_prior_x_auto)/(delta_whycon*100))
	    Kd[1]=Kd[1]+(math.fabs(err_y_auto-err_prior_y_auto)/(delta_whycon*100))
	    err_prior_x_auto=err_x_auto
	    err_prior_y_auto=err_y_auto
	#When pitch,roll and altitude are not damped  
	  elif((math.fabs(max(err_x))-math.fabs(min(err_x)))>1 and (math.fabs(max(err_y))-math.fabs(min(err_y)))>1 and (math.fabs(max(err_z))-math.fabs(min(err_z)))>1.6):
	    r=0
	    err_x_auto=err[a]
	    err_y_auto=err[a+1]
	    err_z_auto=err[a+2]
	    Kd[0]=Kd[0]+(math.fabs(err_x_auto-err_prior_x_auto)/(delta_whycon*100))
	    Kd[1]=Kd[1]+(math.fabs(err_y_auto-err_prior_y_auto)/(delta_whycon*100))
	    Kd[2]=Kd[2]+(math.fabs(err_z_auto-err_prior_z_auto)/(delta_whycon*100))
	    err_prior_x_auto=err_x_auto
	    err_prior_y_auto=err_y_auto
	    err_prior_z_auto=err_z_auto
	  print Kp
	  print Kd

    elif(flag_Kp_start==2): #Ki tuning initiated
        print 'Kp tuned'
        print 'Kd tuning in progress'
        print 'Ki tuning initiated'
        now_ki=time.time()
        if((now_ki-prev_ki)>=1): #Ki value is updated after every 2 seconds
        #When pitch, roll and altitude have offset
          if(math.fabs(err[a])>0.8 and math.fabs(err[a+1])>0.8 and math.fabs(err[a+2])>1):
            r=0
            Ki[0]=Ki[0]+0.25
            Ki[1]=Ki[1]+0.25
            Ki[2]=Ki[2]+0.25
        #When pitch and roll have offset
          elif(math.fabs(err[a])>0.8 and math.fabs(err[a+1])>0.8 and math.fabs(err[a+2])<1):
            r=0
            Ki[0]=Ki[0]+0.25
            Ki[1]=Ki[1]+0.25
        #When roll and altitude have offset
          elif(math.fabs(err[a]<0.8) and math.fabs(err[a+1])>0.8 and math.fabs(err[a+2])>1):
            r=0
            Ki[1]=Ki[1]+0.25
            Ki[2]=Ki[2]+0.25
        #When roll has offset
          elif(math.fabs(err[a]<0.8) and math.fabs(err[a+1])>0.8 and math.fabs(err[a+2])<1):
            r=0
            Ki[1]=Ki[1]+0.25
        #When pitch and altitude have offset
          elif(math.fabs(err[a]>0.8) and math.fabs(err[a+1]<0.8) and math.fabs(err[a+2])>1):
            r=0
            Ki[0]=Ki[0]+0.25
            Ki[2]=Ki[2]+0.25
        #When pitch has offset 
          elif(math.fabs(err[a]>0.8) and math.fabs(err[a+1]<0.8) and math.fabs(err[a+2])<1):
            r=0
            Ki[0]=Ki[0]+0.25
        #When altitude has offset
          elif(math.fabs(err[a]<=0.8) and math.fabs(err[a+1]<=0.8) and math.fabs(err[a+2])>1):
            r=0
            Ki[2]=Ki[2]+0.25
        #When drone is in the target range 
          elif(math.fabs(err[a]<=0.8) and math.fabs(err[a+1]<=0.8) and math.fabs(err[a+2])<=1):
            r=r+1
            if(r==10): #Condition should be true consecutively 10 times
              print 'Auto tuning completed. All good :)'
              print 'Waypoint Navigation Initiated'
              timeout=time.time()+10
              print 'Time elapsed = ',(timeout-10-start_time_tuning)
              flag_Kp_start=3

        #Same process as mentioned above for D is repeated
          if((math.fabs(max(err_x))+math.fabs(min(err_x)))<=1 and (math.fabs(max(err_y))+math.fabs(min(err_y)))<=1 and (math.fabs(max(err_z))+math.fabs(min(err_z)))>1.6):
            err_z_auto=err[a+2]
            Kd[2]=Kd[2]+(math.fabs((err_z_auto)-(err_prior_z_auto))/(delta_whycon*150))
            err_prior_z_auto=err_z_auto
          #When pitch is not damped and roll is
          elif((math.fabs(max(err_x))+math.fabs(min(err_x)))<1 and (math.fabs(max(err_y))+math.fabs(min(err_y)))>1 and (math.fabs(max(err_y))+math.fabs(min(err_y)))<=2 and (math.fabs(max(err_z))+math.fabs(min(err_z)))<1.6):
            err_y_auto=err[a+1]
            Kd[1]=Kd[1]+(math.fabs(err_y_auto-err_prior_y_auto)/(delta_whycon*150))
            err_prior_y_auto=err_y_auto

          elif((math.fabs(max(err_x))+math.fabs(min(err_x)))<1 and (math.fabs(max(err_y))+math.fabs(min(err_y)))>2 and (math.fabs(max(err_z))+math.fabs(min(err_z)))<1.6):
            err_y_auto=err[a+1]
            Kd[1]=Kd[1]-(math.fabs(err_y_auto-err_prior_y_auto)/(delta_whycon*150))
            err_prior_y_auto=err_y_auto

          elif((math.fabs(max(err_x))+math.fabs(min(err_x)))<1 and (math.fabs(max(err_y))+math.fabs(min(err_y)))>1 and (math.fabs(max(err_y))+math.fabs(min(err_y)))<=2 and (math.fabs(max(err_z))+math.fabs(min(err_z)))>1.6):
            err_z_auto=err[a+2]
            err_y_auto=err[a+1]
            Kd[2]=Kd[2]+(math.fabs((err_z_auto-err_prior_z_auto))/(delta_whycon*150))
            Kd[1]=Kd[1]+(math.fabs(err_y_auto-err_prior_y_auto)/(delta_whycon*150))
            err_prior_z_auto=err_z_auto
            err_prior_y_auto=err_y_auto

          elif((math.fabs(max(err_x))+math.fabs(min(err_x)))<1 and (math.fabs(max(err_y))+math.fabs(min(err_y)))>2 and (math.fabs(max(err_z))+math.fabs(min(err_z)))>1.6):
            err_z_auto=err[a+2]
            err_y_auto=err[a+1]
            Kd[2]=Kd[2]+(math.fabs((err_z_auto-err_prior_z_auto))/(delta_whycon*150))
            Kd[1]=Kd[1]-(math.fabs(err_y_auto-err_prior_y_auto)/(delta_whycon*150))
            err_prior_z_auto=err_z_auto
            err_prior_y_auto=err_y_auto

          elif((math.fabs(max(err_x))+math.fabs(min(err_x)))>1 and (math.fabs(max(err_x))+math.fabs(min(err_x)))<=2 and (math.fabs(max(err_y))+math.fabs(min(err_y)))<1 and (math.fabs(max(err_z))+math.fabs(min(err_z)))<1.6):
            err_x_auto=err[a]
            Kd[0]=Kd[0]+(math.fabs(err_x_auto-err_prior_x_auto)/(delta_whycon*150))
            err_prior_x_auto=err_x_auto

          elif((math.fabs(max(err_x))+math.fabs(min(err_x)))>2 and (math.fabs(max(err_y))+math.fabs(min(err_y)))<1 and (math.fabs(max(err_z))+math.fabs(min(err_z)))<1.6):
            err_x_auto=err[a]
            Kd[0]=Kd[0]-(math.fabs(err_x_auto-err_prior_x_auto)/(delta_whycon*150))
            err_prior_x_auto=err_x_auto

          elif((math.fabs(max(err_x))+math.fabs(min(err_x)))>1 and (math.fabs(max(err_x))+math.fabs(min(err_x)))<=2 and (math.fabs(max(err_y))+math.fabs(min(err_y)))<1 and (math.fabs(max(err_z))+math.fabs(min(err_z)))>1.6):
            err_x_auto=err[a]
            err_z_auto=err[a+2]
            Kd[0]=Kd[0]+(math.fabs(err_x_auto-err_prior_x_auto)/(delta_whycon*150))
            Kd[2]=Kd[2]+(math.fabs((err_z_auto-err_prior_z_auto))/(delta_whycon*150))
            err_prior_z_auto=err_z_auto
            err_prior_x_auto=err_x_auto

          elif((math.fabs(max(err_x))+math.fabs(min(err_x)))>2 and (math.fabs(max(err_y))+math.fabs(min(err_y)))<1 and (math.fabs(max(err_z))+math.fabs(min(err_z)))>1.6):
            err_x_auto=err[a]
            err_z_auto=err[a+2]
            Kd[0]=Kd[0]-(math.fabs(err_x_auto-err_prior_x_auto)/(delta_whycon*150))
            Kd[2]=Kd[2]+(math.fabs((err_z_auto-err_prior_z_auto))/(delta_whycon*150))
            err_prior_z_auto=err_z_auto
            err_prior_x_auto=err_x_auto
  
          elif((math.fabs(max(err_x))+math.fabs(min(err_x)))>1 and (math.fabs(max(err_x))+math.fabs(min(err_x)))<=2 and (math.fabs(max(err_y))+math.fabs(min(err_y)))>1 and (math.fabs(max(err_y))+math.fabs(min(err_y)))<=2 and (math.fabs(max(err_z))+math.fabs(min(err_z)))<1.6):
            err_x_auto=err[a]
            err_y_auto=err[a+1]
            Kd[0]=Kd[0]+(math.fabs(err_x_auto-err_prior_x_auto)/(delta_whycon*100))
            Kd[1]=Kd[1]+(math.fabs(err_y_auto-err_prior_y_auto)/(delta_whycon*100))
            err_prior_x_auto=err_x_auto
            err_prior_y_auto=err_y_auto

          elif((math.fabs(max(err_x))+math.fabs(min(err_x)))>2 and (math.fabs(max(err_y))+math.fabs(min(err_y)))>2 and (math.fabs(max(err_z))+math.fabs(min(err_z)))<1.6):
            err_x_auto=err[a]
            err_y_auto=err[a+1]
            Kd[0]=Kd[0]-(math.fabs(err_x_auto-err_prior_x_auto)/(delta_whycon*100))
            Kd[1]=Kd[1]-(math.fabs(err_y_auto-err_prior_y_auto)/(delta_whycon*100))
            err_prior_x_auto=err_x_auto
            err_prior_y_auto=err_y_auto

          elif((math.fabs(max(err_x))+math.fabs(min(err_x)))>1 and (math.fabs(max(err_x))+math.fabs(min(err_x)))<=2 and (math.fabs(max(err_y))+math.fabs(min(err_y)))>1 and (math.fabs(max(err_y))+math.fabs(min(err_y)))<=2 and (math.fabs(max(err_z))+math.fabs(min(err_z)))>1.6):
            err_x_auto=err[a]
            err_y_auto=err[a+1]
            err_z_auto=err[a+2]
            Kd[0]=Kd[0]+(math.fabs(err_x_auto-err_prior_x_auto)/(delta_whycon*100))
            Kd[1]=Kd[1]+(math.fabs(err_y_auto-err_prior_y_auto)/(delta_whycon*100))
            Kd[2]=Kd[2]+(math.fabs(err_z_auto-err_prior_z_auto)/(delta_whycon*100))
            err_prior_x_auto=err_x_auto
            err_prior_y_auto=err_y_auto
            err_prior_z_auto=err_z_auto

          elif((math.fabs(max(err_x))+math.fabs(min(err_x)))>2 and (math.fabs(max(err_y))+math.fabs(min(err_y)))>2 and (math.fabs(max(err_z))+math.fabs(min(err_z)))>1.6):
            err_x_auto=err[a]
            err_y_auto=err[a+1]
            err_z_auto=err[a+2]
            Kd[0]=Kd[0]-(math.fabs(err_x_auto-err_prior_x_auto)/(delta_whycon*100))
            Kd[1]=Kd[1]-(math.fabs(err_y_auto-err_prior_y_auto)/(delta_whycon*100))
            Kd[2]=Kd[2]+(math.fabs(err_z_auto-err_prior_z_auto)/(delta_whycon*100))
            err_prior_x_auto=err_x_auto
            err_prior_y_auto=err_y_auto
            err_prior_z_auto=err_z_auto 
          prev_ki=now_ki
        print Kp
        print Kd
        print Ki

    if(Kp[0]<=Kp_min[0]): #Pitch Kp minimum cap
        Kp[0]=Kp_min[0]
    elif(Kp[0]>=Kp_max[0]): #Pitch Kp maximum cap
        Kp[0]=Kp_max[0]
    if(Kp[1]<=Kp_min[1]): #Roll Kp minimum cap
        Kp[1]=Kp_min[1]
    elif(Kp[1]>=Kp_max[1]): #Roll Kp maximum cap
        Kp[1]=Kp_max[1]
    if(Kp[2]<=Kp_min[2]): #Roll Kp minimum cap
        Kp[2]=Kp_min[2]
    elif(Kp[2]>=Kp_max[2]): #Roll Kp maximum cap
        Kp[2]=Kp_max[2]

    if(Kd[0]>=Kd_max[0]): #Pitch Kd maximum cap
        Kd[0]=Kd_max[0]
    elif(Kd[0]<=Kd_min[0]): #Ptch Kd minimum cap
        Kd[0]=Kd_min[0]
    if(Kd[1]>=Kd_max[1]): #Roll Kd maximum cap
        Kd[1]=Kd_max[1]
    elif(Kd[1]<=Kd_min[1]): #Roll Kd minimum cap
        Kd[1]=Kd_min[1]
    if(Kd[2]>=Kd_max[2]): #Roll Kd maximum cap
        Kd[2]=Kd_max[2]
    elif(Kd[2]<=Kd_min[2]): #Roll Kd minimum cap
        Kd[2]=Kd_min[2]

    if(Ki[0]>=Ki_max[0]): #Pitch Ki maximum cap
        Ki[0]=Ki_max[0]
    elif(Ki[0]<=Ki_min[0]): #Pitch Ki minimum cap
        Ki[0]=Ki_min[0]
    if(Ki[1]>=Ki_max[1]): #Roll Ki maximum cap 
        Ki[1]=Ki_max[1]
    elif(Ki[1]<=Ki_min[1]): #Roll Ki minimum cap
        Ki[1]=Ki_min[1]
    if(Ki[2]>=Ki_max[2]): #Altitude Ki maximum cap
        Ki[2]=Ki_max[2]
    elif(Ki[2]<=Ki_min[2]): #Altitude Ki minimum cap
        Ki[2]=Ki_min[2]  

    if(a==15): #Since maximum size of err array is 18 and value of 'a' is incremented by 3 units
        a=0
    else:
        a=a+3
    if(w==53): #Since maximum size of err_x, err_y and err_z array is 54
        w=0
    else:
        w=w+1

    if(flag_Kp_start==3 and j==1): #If drone is tuned increase count for waypoint after 10 sec of reaching the required point.
      if(math.fabs(err[a]<=0.8) and math.fabs(err[a+1]<=0.8) and math.fabs(err[a+2])<=1):
        if(time.time()>timeout):
          print 'Moving to point ',count
          count=count+1
          if(count==4):
            count=0
            j=2
          timeout=time.time()+10

    err_x_previous=err_x
    err_y_previous=err_y
    err_z_previous=err_z
    prev_whycon=now_whycon

if __name__=="__main__":
    rospy.init_node('drone_server')
    command_pub = rospy.Publisher('/drone_command', PlutoMsg, queue_size=1)
    err_pub_x = rospy.Publisher('/errPublisher_x', Float64, queue_size=1) #Publishing error for graph
    err_pub_y = rospy.Publisher('/errPublisher_y', Float64, queue_size=1)
    err_pub_z = rospy.Publisher('/errPublisher_z', Float64, queue_size=1)
    #rate=rospy.Rate(10)

    rospy.Subscriber("/whycon/poses", PoseArray, callback1) #Subscribed to whycon co-ordinates
    
    cmd = PlutoMsg()
    cmd.rcRoll =1500
    cmd.rcPitch = 1500
    cmd.rcYaw =1500
    cmd.rcThrottle =1500
    cmd.rcAUX1 =1500
    cmd.rcAUX2 =1500
    cmd.rcAUX3 =1500
    cmd.rcAUX4 =1000
    prev_time=time.time()
    print 'Hey there! I am your auto tuning assistant. I will help you get the best PID values for your drone'
    rospy.sleep(2)
    print'Make sure the battery is fully charged'
    rospy.sleep(2)
    Initialisation()
    print 'Pluto is about to be ARMED. Stay clear of the drone.'
    rospy.sleep(1)
    try:
        pass
        # print value()
        while not rospy.is_shutdown():
            #key = getKey()
            if(j==0):
              cmd.rcThrottle =1300
              cmd.rcAUX4 = 1200
              command_pub.publish(cmd)
              rospy.sleep(1)
              cmd.rcRoll=1500 #Arming the code
              cmd.rcYaw=1500
              cmd.rcPitch =1500
              cmd.rcThrottle =1000
              cmd.rcAUX4 =1500
              command_pub.publish(cmd)
              rospy.sleep(1)
              j=1
            elif(j==1): #After three seconds this gets initiated
              print 'Auto tuning initiated'
              start_time_tuning=time.time() #For determining the time for auto tuning.  
              while True:
                  motion()
                  if(j==2): #This condition will only be true after completing waypoint navigation
                    break
                  if(out[2]>max_vel_z):
                    out[2]=max_vel_z
                  elif(out[2]<min_vel_z):
                    out[2]=min_vel_z
                  if(out[1]>max_vel):
                    out[1]=max_vel
                  elif(out[1]<min_vel):
                    out[1]=min_vel
                  if(out[0]>max_vel):
                    out[0]=max_vel
                  elif(out[0]<min_vel):
                    out[0]=min_vel
                  cmd.rcThrottle =1500 - out[2] #Calulating throttle
                  cmd.rcRoll=1500 + out[1] #Calulating roll
                  cmd.rcPitch =1500 + out[0] #Calulating pitch
                  #cmd.rcYaw=1500
                  command_pub.publish(cmd) #For publishing commands to pluto drone
                  err_pub_x.publish(err[0])
                  err_pub_y.publish(err[1])
                  err_pub_z.publish(err[2])                   
            elif(j==2):
              print 'Landing initiated' #Code for landing
              timeout1 = time.time() + 8 #After 8 seconds, disarm the drone
              while True:
                  motion()
                  if(z_co>=27 and (time.time()>=timeout1)):
                    timeout1=0
                    j=3
                    i=1
                    break
                  if(out[1]>max_vel):
                    out[1]=max_vel
                  elif(out[1]<min_vel):
                    out[1]=min_vel
                  if(out[0]>max_vel):
                    out[0]=max_vel
                  elif(out[0]<min_vel):
                    out[0]=min_vel  
                  cmd.rcThrottle =1350    
                  cmd.rcRoll=1500 + out[1] # PID continues for roll
                  cmd.rcPitch =1500 + out[0] # PID continues for pitch
                  cmd.rcYaw=1500
                  command_pub.publish(cmd) #Command for publishing to drone
                  err_pub_x.publish(err[0])
                  err_pub_y.publish(err[1])
                  err_pub_z.publish(err[2])
            elif(i==1):
              cmd.rcThrottle =1300 #Code for dis arming
              cmd.rcAUX4 = 1200
              command_pub.publish(cmd)
              rospy.sleep(1)

    except Exception as e:
        print e