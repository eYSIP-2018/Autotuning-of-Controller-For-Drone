#!/usr/bin/env python
from plutodrone.srv import *
from plutodrone.msg import *
from std_msgs.msg import Int16
from std_msgs.msg import Int64
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseArray
from pid_tune.msg import PidTune
import rospy
import time


# class drone_control to control drone navigation
class drone_control():
	def __init__(self):
		rospy.init_node('control_drone')

		# point to hover 
		self.dest = [0, 0, 20]

		# Kp values of Pitch, Roll, Throttle
		self.pitch_kp =10
		self.roll_kp = 10
		self.throttle_kp = 10

		# drone location initialisation
		self.drone = [0.0,0.0,0.0]

		# max values of Pitch, Roll, Throttle
		self.rc_roll_max =1575
		self.rc_pitch_max=1575
		self.rc_throttle_max=1800

		# max values of Pitch, Roll, Throttle
		self.rc_roll_min =1425
		self.rc_pitch_min = 1425
		self.rc_throttle_min =1200

		# drone parameters
		self.cmd = PlutoMsg()
		self.cmd.rcRoll = 1500
		self.cmd.rcPitch = 1500 
		self.cmd.rcYaw = 1500
		self.cmd.rcThrottle = 1500
		self.cmd.rcAUX1 = 1500
		self.cmd.rcAUX2 = 1500
		self.cmd.rcAUX3 = 1500
		self.cmd.rcAUX4 = 1000

		# initialise variable prev_time 
		self.prev_time = time.time()

		#initialise sample time
		self.sample_time = 0.030
		

		# a publisher that publishes on the topic /drone_command 
		self.command_pub = rospy.Publisher('/drone_command', PlutoMsg, queue_size=1)

		# instantiating a subscriber to subscribe to topic /whycon/poses that gives the drone coordinates
		rospy.Subscriber('/whycon/poses',PoseArray,self.drone_location)
		
		# publishers that publish errors ; used to plot graphs
		self.x_err_pub = rospy.Publisher('/y_error', Float64, queue_size=1)
		self.y_err_pub = rospy.Publisher('/x_error', Float64, queue_size=1)
		self.z_err_pub = rospy.Publisher('/z_error', Float64, queue_size=1)

		# arm the drone
		self.arm()
		
	# method to arm the drone
	def arm(self):
		self.cmd.rcRoll = 1500
		self.cmd.rcYaw = 1500
		self.cmd.rcPitch = 1500
		self.cmd.rcThrottle = 1000
		self.cmd.rcAUX4 = 1500
		self.command_pub.publish(self.cmd)
		rospy.sleep(0.1)

	# method to disarm the drone
	def disarm(self):
		self.cmd.rcThrottle = 1300
		self.cmd.rcAUX4 = 1200
		self.command_pub.publish(self.cmd)
		rospy.sleep(0.1)

	# method to get the drone
	def drone_location(self,msg):

		# holds the x-coordinate
		self.drone[0] = msg.poses[0].position.x

		#holds the y-coordinate
		self.drone[1] = msg.poses[0].position.y

		#holds the z-coordinate
		self.drone[2] = msg.poses[0].position.z
		
	# method that brings about stable drone navigation
	def pid(self):

		# difference in x-coordinate of detsination and drone 
		error_x = self.dest[0]- self.drone[0]

		# difference in y-coordinate of detsination and drone 
		error_y = self.dest[1]- self.drone[1]

		# difference in z-coordinate of detsination and drone 
		error_z = self.dest[2]- self.drone[2]

		# current time
		curr_time = time.time()

		# variable that holds the time lapse 
		dt = curr_time - self.prev_time

		# if time lapse less than the sample tiime ; exit
		if (dt< self.sample_time):
			return
		
		# else ; compute pid output
		else:
			# update previous time
			self.prev_time = curr_time

			#publish the errors
			self.x_err_pub.publish(error_x)
			self.y_err_pub.publish(error_y)
			self.z_err_pub.publish(error_z)

			# determine the pid output on roll, pitch, throttle axes
			self.out_roll = (self.roll_kp * error_y) 
			self.out_pitch = (self.pitch_kp * error_x)
			self.out_throttle= (self.throttle_kp * error_z)


			# compute the values that will be published on topic /drone_command
			self.cmd.rcRoll= 1500  + self.out_roll
			self.cmd.rcPitch =1500 + self.out_pitch
			self.cmd.rcThrottle = 1500 - self.out_throttle

			# cap the values if they exceed the pre determined limits
			if self.cmd.rcThrottle > self.rc_throttle_max:
				self.cmd.rcThrottle=self.rc_throttle_max
			elif self.cmd.rcThrottle < self.rc_throttle_min:
				self.cmd.rcThrottle = self.rc_throttle_min

			if self.cmd.rcRoll > self.rc_roll_max:
				self.cmd.rcRoll=self.rc_roll_max
			elif self.cmd.rcRoll < self.rc_roll_min:
				self.cmd.rcRoll = self.rc_roll_min

			if self.cmd.rcPitch > self.rc_pitch_max:
				self.cmd.rcPitch = self.rc_pitch_max
			elif self.cmd.rcPitch < self.rc_pitch_min:
				self.cmd.rcPitch = self.rc_pitch_min


			# publish the computesd values
			self.command_pub.publish(self.cmd)
		


if __name__ == '__main__':

	# initialise object fly
	fly=drone_control()

	# position holding
	while not rospy.is_shutdown():
		fly.pid()