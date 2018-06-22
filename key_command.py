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
#Initially Kp, Kd & Ki are set to minimum. After the drone gets armed, Kp and Kd increase wrt rate of change of error.
#After the drone crosses a circle of radius 1.8 units three times, Kp is set constant. Kd is increased again wrt rate
#of change of error till twice amplitude is less than 1 unit. The mentioned condition should be true for 10 cycles.
#Then Ki is increased till the drones maintains itself in a position of 0.8 units circle. Again this conditon is
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
global timeout
i=0
j=0
timeout=0
count=1

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
Mp_max=0
Mp_min=0
flag_reduce_Kp=0
flag_Kp_start=0
sumerr=[0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
sumerr1=[0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
max_vel=150
min_vel=-150
max_vel_z=250
min_vel_z=-250
Kp=[0.0,0.0,0.0]
Ki=[0.0,0.0,0.0]
Kd=[0.0,0.0,0.0]

Kp_max=[22,22,40]
Kp_min=[5,5,20]

Kd_max=[20,20,8]
Kd_min=[3,3,0]

Ki_max=[50,50,10]
Ki_min=[0,0,0]

kp_auto_x=Kp_min[0]
kp_auto_y=Kp_min[1]
kp_auto_z=Kp_max[2]

ki_auto_x=Ki_min[0]
ki_auto_y=Ki_min[1]
ki_auto_z=Ki_min[2]

kd_auto_x=Kd_min[0]
kd_auto_y=Kd_min[1]
kd_auto_z=Kd_min[2]

list_co=[0,0,16]

def callback1(msg):
  global x_co
  global y_co
  global z_co
  x_co=msg.poses[0].position.x
  y_co=msg.poses[0].position.y
  z_co=msg.poses[0].position.z

def motion():
  global count
  global err
  global err_x
  global err_y
  global err_z
  global Kp
  global Ki
  global Kd
  global flag_initialize
  if(flag_initialize==0): # Initial values are set with the help of this flag.
    Kp=[kp_auto_x,kp_auto_y,kp_auto_z]
    Ki=[ki_auto_x,ki_auto_y,ki_auto_z]
    Kd=[kd_auto_x,kd_auto_y,kd_auto_z]
    err[a]=list_co[0]-x_co
    err[a+1]=list_co[1]-y_co
    err[a+2]=list_co[2]-z_co
    flag_initialize=1
    PID(err)
  else:
    err[a]=list_co[0]-x_co #The zeroth, third, sixth..fifteenth values of the array store pitch error
    err[a+1]=list_co[1]-y_co #Similarly first, fourth, seventh...sixteenth values of the array store roll error
    err[a+2]=list_co[2]-z_co

    err_x[w]=list_co[0]-x_co #Array of 54 elements to store pitch error. This is used to find max and min values  
    err_y[w]=list_co[1]-y_co #Array of 54 elements to store roll error. This is used to find max and min values
    err_z[w]=list_co[2]-z_co #Array of 54 elements to store throttle error. This is used to find max and min values
    PID(err)

def PID(err):
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
  global flag_reduce_Kp
  global flag_Kp_start
  global now_whycon
  global prev_whycon
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
  global sign_x
  global sign_y
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
  if(delta_whycon>0.030): #Rate of whycon co-ordinates
    if(flag_Kp_start==0): #For Finding value of Kp
      print 'Kp tuning initiated'
      if(math.fabs(err[a])<1.8 and math.fabs(err[a+1])>1.8 and math.fabs(err[a+2])>2): #When pitch co-ordiinate is within limits and roll is not
        r=0
        err_x_auto=err[a]
        err_y_auto=err[a+1]
        err_z_auto=err[a+2]
        Kp[1]=Kp[1]+0.1 #rate of change of error
        Kp[2]=Kp[2]-(err_z_auto-err_prior_z_auto)/(delta_whycon*100)
        # Kd[2]=Kd[2]+(err_z_auto-err_prior_z_auto)/(delta_whycon*100)
        # Kd[1]=Kd[1]+(err_y_auto-err_prior_y_auto)/(delta_whycon*100)
        err_prior_x_auto=err_x_auto
        err_prior_y_auto=err_y_auto
        err_prior_z_auto=err_z_auto
      elif(math.fabs(err[a])<1.8 and math.fabs(err[a+1])>1.8 and math.fabs(err[a+2])<2): #When pitch co-ordiinate is within limits and roll is not
        r=0
        err_x_auto=err[a]
        err_y_auto=err[a+1]
        err_z_auto=err[a+2]
        Kp[1]=Kp[1]+0.1 #rate of change of error
        # Kd[1]=Kd[1]+(err_y_auto-err_prior_y_auto)/(delta_whycon*100)
        err_prior_x_auto=err_x_auto
        err_prior_y_auto=err_y_auto
        err_prior_z_auto=err_z_auto
      elif(math.fabs(err[a])>1.8 and math.fabs(err[a+1])<1.8 and math.fabs(err[a+2])>2):
        r=0
        err_x_auto=err[a]
        err_y_auto=err[a+1]
        err_z_auto=err[a+2]
        Kp[0]=Kp[0]+0.1
        Kp[2]=Kp[2]-(err_z_auto-err_prior_z_auto)/(delta_whycon*100)
        # Kd[2]=Kd[2]+(err_z_auto-err_prior_z_auto)/(delta_whycon*100)
        # Kd[0]=Kd[0]+(err_x_auto-err_prior_x_auto)/(delta_whycon*100)
        err_prior_x_auto=err_x_auto
        err_prior_y_auto=err_y_auto
        err_prior_z_auto=err_z_auto
      elif(math.fabs(err[a])>1.8 and math.fabs(err[a+1])<1.8 and math.fabs(err[a+2])<2):
        r=0
        err_x_auto=err[a]
        err_y_auto=err[a+1]
        err_z_auto=err[a+2]
        Kp[0]=Kp[0]+0.1
        # Kd[0]=Kd[0]+(err_x_auto-err_prior_x_auto)/(delta_whycon*100)
        err_prior_x_auto=err_x_auto
        err_prior_y_auto=err_y_auto
        err_prior_z_auto=err_z_auto
      elif(math.fabs(err[a])>1.8 and math.fabs(err[a+1])>1.8 and math.fabs(err[a+2])>2):
        r=0
        err_x_auto=err[a]
        err_y_auto=err[a+1]
        err_z_auto=err[a+2]
        Kp[0]=Kp[0]+(err_x_auto-err_prior_x_auto)/(delta_whycon*30)
        Kp[1]=Kp[1]+(err_y_auto-err_prior_y_auto)/(delta_whycon*30)
        Kp[2]=Kp[2]-(err_z_auto-err_prior_z_auto)/(delta_whycon*100)
        # Kd[2]=Kd[2]+(err_z_auto-err_prior_z_auto)/(delta_whycon*100)
        # Kd[0]=Kd[0]+(err_x_auto-err_prior_x_auto)/(delta_whycon*100)
        # Kd[1]=Kd[1]+(err_y_auto-err_prior_y_auto)/(delta_whycon*100)
        err_prior_x_auto=err_x_auto
        err_prior_y_auto=err_y_auto
        err_prior_z_auto=err_z_auto
      elif(math.fabs(err[a])>1.8 and math.fabs(err[a+1])>1.8 and math.fabs(err[a+2])<2):
        r=0
        err_x_auto=err[a]
        err_y_auto=err[a+1]
        Kp[0]=Kp[0]+(err_x_auto-err_prior_x_auto)/(delta_whycon*30)
        Kp[1]=Kp[1]+(err_y_auto-err_prior_y_auto)/(delta_whycon*30)
        # Kd[0]=Kd[0]+(err_x_auto-err_prior_x_auto)/(delta_whycon*100)
        # Kd[1]=Kd[1]+(err_y_auto-err_prior_y_auto)/(delta_whycon*100)
        err_prior_x_auto=err_x_auto
        err_prior_y_auto=err_y_auto
      elif(math.fabs(err[a])<=1.8 and math.fabs(err[a+1])<=1.8 and math.fabs(err[a+2])>2):
        r=0
        err_x_auto=err[a]
        err_y_auto=err[a+1]
        err_z_auto=err[a+2]
        # Kp[0]=Kp[0]+(err_x_auto-err_prior_x_auto)/(delta_whycon*100)
        # Kp[1]=Kp[1]+(err_y_auto-err_prior_y_auto)/(delta_whycon*100)
        Kp[2]=Kp[2]-(err_z_auto-err_prior_z_auto)/(delta_whycon*100)
        #Kd[2]=Kd[2]+(err_z_auto-err_prior_z_auto)/(delta_whycon*100)
        # Kd[0]=Kd[0]+(err_x_auto-err_prior_x_auto)/(delta_whycon*100)
        # Kd[1]=Kd[1]+(err_y_auto-err_prior_y_auto)/(delta_whycon*100)
        err_prior_x_auto=err_x_auto
        err_prior_y_auto=err_y_auto
        err_prior_z_auto=err_z_auto

      elif(math.fabs(err[a])<=1.8 and math.fabs(err[a+1])<=1.8 and math.fabs(err[a+2])<=2):
        r=r+1
        if(r==10): # To check the condition 10 times
          flag_Kp_start=1
          r=0
      print Kp
      print Kd
    elif(flag_Kp_start==1): #Kd tuning initiated
        print 'Kp tuned'
        print 'Kd tuning initiated'
        now_ki=time.time()
        if((now_ki-prev_ki)>=1): #Value is updated after every 2 seconds
          if((math.fabs(max(err_x))-math.fabs(min(err_x)))<=0.8 and (math.fabs(max(err_y))-math.fabs(min(err_y)))<=0.8 and (math.fabs(max(err_z))-math.fabs(min(err_z)))>1):
            err_z_auto=err[a+2]
            Kd[2]=Kd[2]+(err_z_auto-err_prior_z_auto)/(delta_whycon*100)
            err_prior_z_auto=err_z_auto
          elif((math.fabs(max(err_x))-math.fabs(min(err_x)))<=0.8 and (math.fabs(max(err_y))-math.fabs(min(err_y)))<=0.8 and (math.fabs(max(err_z))-math.fabs(min(err_z)))<=1):
          #Twice the amplitude is less than 0.5 units.
            r=r+1
            if(r==10): #Condition checked 5 times
              r=0
              flag_Kp_start=2
          #When pitch is not damped and roll is
          elif((math.fabs(max(err_x))-math.fabs(min(err_x)))>0.8 and (math.fabs(max(err_y))-math.fabs(min(err_y)))<0.8 and (math.fabs(max(err_z))-math.fabs(min(err_z)))<1):
            r=0
            Kd[0]=Kd[0]+0.2
          elif((math.fabs(max(err_x))-math.fabs(min(err_x)))>0.8 and (math.fabs(max(err_y))-math.fabs(min(err_y)))<0.8 and (math.fabs(max(err_z))-math.fabs(min(err_z)))>1):
            r=0
            err_z_auto=err[a+2]
            Kd[0]=Kd[0]+0.2
            Kd[2]=Kd[2]+(err_z_auto-err_prior_z_auto)/(delta_whycon*100)
            err_prior_z_auto=err_z_auto
          elif((math.fabs(max(err_x))-math.fabs(min(err_x)))<0.8 and (math.fabs(max(err_y))-math.fabs(min(err_y)))>0.8 and (math.fabs(max(err_z))-math.fabs(min(err_z)))<1):
            r=0
            Kd[1]=Kd[1]+0.2
          elif((math.fabs(max(err_x))-math.fabs(min(err_x)))<0.8 and (math.fabs(max(err_y))-math.fabs(min(err_y)))>0.8 and (math.fabs(max(err_z))-math.fabs(min(err_z)))>1):
            r=0
            Kd[0]=Kd[0]+0.2
            Kd[1]=Kd[1]+0.2
          elif((math.fabs(max(err_x))-math.fabs(min(err_x)))>0.8 and (math.fabs(max(err_y))-math.fabs(min(err_y)))>0.8 and (math.fabs(max(err_z))-math.fabs(min(err_z)))<1):
            r=0
            err_x_auto=err[a]
            err_y_auto=err[a+1]
            Kd[0]=Kd[0]+(err_x_auto-err_prior_x_auto)/(delta_whycon*60)
            Kd[1]=Kd[1]+(err_y_auto-err_prior_y_auto)/(delta_whycon*60)
            err_prior_x_auto=err_x_auto
            err_prior_y_auto=err_y_auto
          elif((math.fabs(max(err_x))-math.fabs(min(err_x)))>0.8 and (math.fabs(max(err_y))-math.fabs(min(err_y)))>0.8 and (math.fabs(max(err_z))-math.fabs(min(err_z)))>1):
            r=0
            err_x_auto=err[a]
            err_y_auto=err[a+1]
            err_z_auto=err[a+2]
            Kd[0]=Kd[0]+(err_x_auto-err_prior_x_auto)/(delta_whycon*60)
            Kd[1]=Kd[1]+(err_y_auto-err_prior_y_auto)/(delta_whycon*60)
            Kd[2]=Kd[2]+(err_z_auto-err_prior_z_auto)/(delta_whycon*100)
            err_prior_x_auto=err_x_auto
            err_prior_y_auto=err_y_auto
            err_prior_z_auto=err_z_auto
          prev_ki=now_ki
        print Kp
        print Kd

    elif(flag_Kp_start==2): #Ki tuning initiated
        print 'Kp tuned'
        print 'Kd tuned'
        print 'Ki tuning initiated'
        now_ki=time.time()
        if((now_ki-prev_ki)>=1): #Ki value is updated after every 2 seconds
          if(math.fabs(err[a])>0.8 and math.fabs(err[a+1])>0.8 and math.fabs(err[a+2])>1):
            r=0
            Ki[0]=Ki[0]+0.25
            Ki[1]=Ki[1]+0.25
            Ki[2]=Ki[2]+0.25
          elif(math.fabs(err[a])>0.8 and math.fabs(err[a+1])>0.8 and math.fabs(err[a+2])<1):
            r=0
            Ki[0]=Ki[0]+0.25
            Ki[1]=Ki[1]+0.25
          elif(math.fabs(err[a]<0.8) and math.fabs(err[a+1])>0.8 and math.fabs(err[a+2])>1):
            r=0
            Ki[1]=Ki[1]+0.25
            Ki[2]=Ki[2]+0.25
          elif(math.fabs(err[a]<0.8) and math.fabs(err[a+1])>0.8 and math.fabs(err[a+2])<1):
            r=0
            Ki[1]=Ki[1]+0.25
          elif(math.fabs(err[a]>0.8) and math.fabs(err[a+1]<0.8) and math.fabs(err[a+2])>1):
            r=0
            Ki[0]=Ki[0]+0.25
            Ki[2]=Ki[2]+0.25
          elif(math.fabs(err[a]>0.8) and math.fabs(err[a+1]<0.8) and math.fabs(err[a+2])<1):
            r=0
            Ki[0]=Ki[0]+0.25
          elif(math.fabs(err[a]<=0.8) and math.fabs(err[a+1]<=0.8) and math.fabs(err[a+2])>1):
            r=0
            Ki[2]=Ki[2]+0.25
          elif(math.fabs(err[a]<=0.8) and math.fabs(err[a+1]<=0.8) and math.fabs(err[a+2])<=1):
            r=r+1
            if(r==5): #Condition is checked 5 times
              print 'All good'
              print 'Have a nice day !! :)'
              flag_Kp_start=3
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

    if(Ki[0]>=Ki_max[0]): 
        Ki[0]=Ki_max[0]
    elif(Ki[0]<=Ki_min[0]):
        Ki[0]=Ki_min[0]
    if(Ki[1]>=Ki_max[1]):
        Ki[1]=Ki_max[1]
    elif(Ki[1]<=Ki_min[1]):
        Ki[1]=Ki_min[1]
    if(Ki[2]>=Ki_max[2]):
        Ki[2]=Ki_max[2]
    elif(Ki[2]<=Ki_min[2]):
        Ki[2]=Ki_min[2]  

    if(a==15): #Since maximum size of err array is 18
        a=0
    else:
        a=a+3
    if(w==53): #Since maximum size of err_x, err_y and err_z array is 54
        w=0
    else:
        w=w+1  
    # print Kp
    # print Kd
    # print Ki
    # print flag_Kp_start
    err_x_previous=err_x
    err_y_previous=err_y
    err_z_previous=err_z
    prev_whycon=now_whycon
    #print flag_reduce_Kp_x

if __name__=="__main__":
    rospy.init_node('drone_server')
    command_pub = rospy.Publisher('/drone_command', PlutoMsg, queue_size=1)
    err_pub_x = rospy.Publisher('/errPublisher_x', Float64, queue_size=1) #Publishing error for graph
    err_pub_y = rospy.Publisher('/errPublisher_y', Float64, queue_size=1)
    err_pub_z = rospy.Publisher('/errPublisher_z', Float64, queue_size=1)
    rate=rospy.Rate(10)
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
    try:
        pass
        # print value()
        while not rospy.is_shutdown():
            #key = getKey()
            if(j==0):
              timeout = time.time() + 3 #For 3 seconds the drone is armed
              while True: 
                 if (time.time() > timeout):
                   j=1
                   timeout=0
                   print 'Pluto has been ARMED'
                   break
                 cmd.rcRoll=1500
                 cmd.rcYaw=1500
                 cmd.rcPitch =1500
                 cmd.rcThrottle =1000
                 cmd.rcAUX4 =1500
                 command_pub.publish(cmd)
            elif(j==1): #After three seconds this gets initiated
              print 'Auto tuning initiated'
              while True:
                  motion()
                  #if (time.time() > timeout):
                  #  timeout=0
                  #  i=1
                  #  break
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
                  cmd.rcThrottle =1500 - out[2]    
                  cmd.rcRoll=1500 - out[1]
                  cmd.rcPitch =1500 - out[0]
                  cmd.rcYaw=1500
                  command_pub.publish(cmd)
                  #print sumerr[0]
                  # print 'rcThrottle'
                  # print cmd.rcThrottle
                  # print 'rcRoll'
                  # print cmd.rcRoll
                  # print 'rcPitch'
                  # print cmd.rcPitch
                  err_pub_x.publish(err[0])
                  err_pub_y.publish(err[1])
                  err_pub_z.publish(err[2])                   
            # elif(i==1):
            #   while True:
            #      timeout = time.time() + 1
            #      if (time.time() > timeout):
            #        i=0
            #        timeout=0
            #        rospy.sleep(.1)
            #        break
            #      cmd.rcThrottle =1300
            #      cmd.rcAUX4 = 1200
            #      command_pub.publish(cmd)
            # #pub.publish(msg_pub)
            # #rate.sleep()

    except Exception as e:
        print e