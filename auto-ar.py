#!/usr/bin/env python
import rospy
import roslib
import time
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Empty
from std_msgs.msg import Float64
import time

# class drone_control to control drone navigation
class drone_control():
	def __init__(self):

		# initialise a node
		rospy.init_node('pos')

		# waypoints to be travelled
		self.dest = [ (0.0, 0.0, 9) , (3, 3, 11),(-3, 3, 11), (-3, -3, 11),(3, -3, 11), (0, 0, 11) ]
		self.length = len(self.dest)

		# a counter that keeps track of waypoint traversed
		self.count =0

		# drone's current location
		self.drone = [0.0, 0.0, 0.0]

		# initialise a variable that will inidcate if a waypoint is succesfully traversed
		self.done= False

		# create a list for KP, Ki, Kd for motion along all three x, y and z axes
		self.kp = [None, None, None]
		self.ki = [None, None, None]
		self.kd = [None, None, None]
		
		# initialise the output
		self.out = [0,0,0]

		# initialise the Iterm to sum up errors in all three axes
		self.Iterm = [0.0, 0.0, 0.0]

		# initialise sample time; time interval for calling the PID function
		self.sample_time =0.015

		# initialise AR Drone parameters
		self.twist = Twist()
		self.twist.linear.x = 0  # linear velocity along x-axis
		self.twist.linear.y = 0	 # linear velocity along y-axis
		self.twist.linear.z = 0	 # linear velocity along z-axis


		# initialise attributes to keep track of current errors and previous errors
		self.error =[0.0, 0.0, 0.0]
		self.prev_error = [0.0,0.0,0.0]

		# initialise a Publisher that will publish data on the topic /cmd_vel
		self.pub_twist = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

		# initialise a publisher that will publish data on the topic /ardrone/takeoff
		self.pub_empty_takeoff = rospy.Publisher('/ardrone/takeoff', Empty, queue_size=1)

		# initialise a publisher that will publish data on the topic /ardrone/takeoff
		self.pub_empty_landing = rospy.Publisher('/ardrone/land', Empty,queue_size=1)

		# initialise publishers that will publish the errors along all three axes i.e x, t and z axes
		self.error_x= rospy.Publisher('error_x',Float64,queue_size=1)
		self.error_y= rospy.Publisher('error_y',Float64,queue_size=1)
		self.error_z= rospy.Publisher('error_z',Float64,queue_size=1)

		# initialise a sunscriber that is going to subscribe to the topic /whycon/poses that will give the drone's current location 
		rospy.Subscriber("/whycon/poses",PoseArray,self.get_data)

		# initialise an attribute that holds previous time
		self.prev_time = time.time()


	# method to access the drone's current location 
	def get_data(self,msg):
		self.drone[0] = msg.poses[0].position.x  # drone's x coordinate
		self.drone[1] = msg.poses[0].position.y  # drone's y coordinate
		self.drone[2] = msg.poses[0].position.z  # drone's z coordinate


	# metod to cap the PID output
	# input --> PID output
	def cap(self,input):
		if input > 1.2:  # upper bound on linear velocity
			return 1.2
		elif input < -1.2: # lower bound on linear velocity
			return -1.2
		else:
			return input


	# method to determine the PID output
	def pid(self):
		# determining errors across all axes
		self.error[0] = self.dest[self.count][0]- self.drone[0]  # error in x-coordinate
		self.error[1] = self.dest[self.count][1]- self.drone[1]	 # error in y-coordinate
		self.error[2] = self.dest[self.count][2]- self.drone[2]	 # error in z-coordinate

		# note the cuurent time
		curr_time = time.time()

		# variable that holds the time lapse 
		dt = curr_time - self.prev_time

		# if time lapse less than the sample tiime ; exit
		if (dt < self.sample_time):
			return

		# update previous time
		self.prev_time = time.time()

		# if the waypoint is in permissible error limits of the waypoint
		if abs(self.error[0])<= 0.03 and abs(self.error[1]) <= 0.03 and abs(self.error[2]) <= 0.03:
			self.done= True # update the counter to indicate waypoint reached
			return

		# compute the Iterm
		# The error is scaled and added to Iterm so as to see that the Iterm does not become large in short time
		# The other alternative to handle is to cap the Iterm as required.
		self.Iterm[0] = self.Iterm[0] + (self.error[0]/100)
		self.Iterm[1] = self.Iterm[1] + (self.error[1]/100)
		self.Iterm[2] = self.Iterm[2] + (self.error[2]/100)
		
		# publish the errors on all three axes
		self.error_x.publish(self.error[0])
		self.error_y.publish(self.error[1])
		self.error_z.publish(self.error[2])

		# determine the pid output on
		self.out[0] = ( self.error[0] * self.kp[0] ) + ( self.Iterm[0] * self.ki[0] ) + ( self.kd[0] * (self.error[0] -self.prev_error[0]) )
		self.out[1] = ( self.error[1] * self.kp[1] ) + ( self.Iterm[1] * self.ki[1] ) + ( self.kd[1] * (self.error[1] -self.prev_error[1]) )
		self.out[2] = ( self.error[2] * self.kp[2] ) + ( self.Iterm[2] * self.ki[2] ) + ( self.kd[2] * (self.error[2] -self.prev_error[2]) )

		# compute the values that will be published on topic /cmd_vel
		self.twist.linear.x = 0 - self.out[1]
		self.twist.linear.y = 0 - self.out[0]
		self.twist.linear.z = 0 - self.out[2]

		# call the cap method to make sure that the values that will be published are within bounds		self.twist.linear.y = self.cap(self.twist.linear.y)
		self.twist.linear.z = self.cap(self.twist.linear.z)

		# update previous errors
		self.prev_error[0] = self.error[0]
		self.prev_error[1] = self.error[1]
		self.prev_error[2] = self.error[2]

		# publish the values
		self.pub_twist.publish(self.twist)



# class AutoTune used to find the PID constants.
# Attributes:
# setpoint --> point at which the drone is to be stabilised
# sample_time --> sampling time used in PID 
# max_val --> maximum output of the PID
# min_val --> minimum output of the PID
# tuning_cycles  --> number of oscillations 
# z_flag --> to indicate if this class is bieng used for z_axis
class AutoTune():

	def __init__(self, setpoint, sample_time, max_val, min_val, tuning_cycles ,z_flag):
		self.setpoint = setpoint
		self.sample_time = sample_time
		self.max_out = max_val
		self.min_out = min_val
		self.tuning_cycles = tuning_cycles
		self.positive_cycle = True 		# flag that specifies if the cycle is positive or negative
		self.out = 0
		self.prev_time =time.time()
		self.flag = z_flag

	# method TuningLoop ; to instantiate the tuning process and initialise the required variables
	def startTuningLoop(self):

		# counter to count no. of oscillations
		self.cycle_count = 0

		# output that will be published 
		self.out = self.max_out

		# initialise timers to keep track of time at positive and neagtive peaks of oscillation
		self.t1 = self.t2 = time.time()
		self.tLow = self.tHigh = 0

		# max and min possible inputs
		self.max = -5
		self.min = 5

		# variables to hold avg values
		self.p_avg = self.i_avg = self.d_avg = 0

	# method tune that brings about tuning of the parameters
	# attributes
	# input --> drone's current location
	def tune(self, input):
		
		# find the max and min value after the following comparison
		self.max = max(self.max, input)
		self.min = min(self.min, input)

		# if the drone is above the setpoint
		if self.positive_cycle:
			# if called by z-axis linear velocity
			# as the z_coordinate changes by +/- 1.5(max) consider this offset / not so much in simulation though
			if self.flag and input > (self.setpoint + 1.5):

				# input > setpoint implies the drone is ahead of the setpoint
				# set this flag to False to make it go to negative cycle
				self.positive_cycle = False

				# Force the output to minimum to move your drone below the setpoint
				self.out = self.min_out

				# note the time at positive half
				self.t1 = time.time()

				# time period of positive cycle
				self.tHigh = self.t1 - self.t2

			# if z_flag is false it means the x linear velocity or y linear velocity autotuning is in progress hence there is no need of offset
			if not self.flag and input > self.setpoint:
				#logic remains same as above
				self.positive_cycle = False
				self.out = self.min_out
				self.t1 = time.time()
				self.tHigh = self.t1 - self.t2

		# if oscilation has entered negative cycle 
		if not self.positive_cycle:

			# check z_flag ; If z_flag is set then it means z linear velocity autotuning is in progress
			if self.flag and input < (self.setpoint - 1.5):

				# set this flag to False to make it go to negative cycle
				self.positive_cycle = True

				# Force the output to maximum to move your drone above the setpoint
				self.out = self.max_out

				#note the time negative cycle was completed
				self.t2 = time.time()

				# time period of negative cycle = current time - time taken to complete positive cycle
				self.tLow = self.t2 - self.t1

				# calculated using the formula ku= 4*d/pi*a
				# d --> amplitude of the output (pid output here)
				# a --> amplitude of the input  ( drone coordinate )
				# d= ((self.max_out - self.min_out) / 2.0))
				# a= (self.max - self.min) / 2.0)
				self.ku = (4.0 * ((self.max_out - self.min_out) / 2.0)) / (3.142 * (self.max - self.min) / 2.0)
				
				#tu --> ultimate time period i.e timpe period of the oscillation; sum pf time period of pos and neg cycles.
				self.tu = self.tLow+self.tHigh
				
				# Ziegler nichols formula for classical PID
				self.kp = 0.8 * self.ku
				self.ki = (self.kp / (0.5 * self.tu)) * self.sample_time
				self.kd = (0.125 * self.kp * self.tu) / self.sample_time

				if self.cycle_count:
					# to detrmine the average of the values over the total nimber of ocsillations
					self.p_avg += self.kp
					self.i_avg += self.ki
					self.d_avg += self.kd

				# after the  oscillation initialise the min value to setpoint 
				self.min = self.setpoint

				# increment the cycle counter
				self.cycle_count= self.cycle_count +1
				print self.cycle_count

			# same logic as above ; for x and y linear velocities; z_flag =false
			if not self.flag and input < self.setpoint:
				self.positive_cycle = True
				self.out = self.max_out
				self.t2 = time.time()
				self.tLow = self.t2 - self.t1
				self.ku = (4.0 * ((self.max_out - self.min_out) / 2.0)) / (3.142 * (self.max - self.min) / 2.0)
				self.tu = self.tLow+self.tHigh
				self.kp = 0.8 * self.ku
				print self.kp
				self.ki = (self.kp / (0.5 * self.tu)) * self.sample_time
				self.kd = (0.25 * self.kp * self.tu) / self.sample_time

				if self.cycle_count:
					self.p_avg += self.kp
					self.i_avg += self.ki
					self.d_avg += self.kd

				self.min = self.setpoint
				self.cycle_count= self.cycle_count +1
				print self.cycle_count

		if self.cycle_count >= self.tuning_cycles:
			self.positive_cycle = False
			self.out = self.min_out

			self.kp = self.p_avg/(self.cycle_count-1)
			self.ki = self.i_avg/(self.cycle_count-1)
			self.kd = self.d_avg/(self.cycle_count-1)


		
		# return the output that will be published to /cmd_vel
		return self.out

if __name__ == '__main__':
	fly = drone_control()

	# initialise the object pid_throttle to autotune the z_vel params
	pid_z_vel = AutoTune(fly.dest[0][2], fly.sample_time, 1.0, -1.0, 3,True)

	# initialise the object pid_pitch to autotune the x_vel params
	pid_x_vel    = AutoTune(fly.dest[0][0], fly.sample_time, 1.0, -1.0 ,3,False)

	# initialise the object pid_pitch to autotune the y_vel params
	pid_y_vel    = AutoTune(fly.dest[0][1], fly.sample_time, 1.0, -1.0, 3,False)
	

	# call the method startTuningLoop to start tuning procedure on respective axis
	pid_z_vel.startTuningLoop()
	pid_x_vel.startTuningLoop()
	pid_y_vel.startTuningLoop()

	print '--------------------------------auto-tuning started---------------------------------------------'

	# to carry out oscillations thehreby determining the KP , KI and KD vaues of throttle,pitch and roll
	while pid_z_vel.cycle_count <3:
		fly.pub_empty_takeoff.publish(Empty())
		z_out = pid_z_vel.tune(fly.drone[2])
		fly.twist.linear.z = 0 - z_out
		fly.twist.linear.x = 0
		fly.twist.linear.y = 0
		fly.pub_twist.publish(fly.twist)
	print 'finished throttle'
	
	


	while pid_x_vel.cycle_count <3:
		x_out = pid_x_vel.tune(fly.drone[0])
		fly.twist.linear.z = 0
		fly.twist.linear.x = 0
		fly.twist.linear.y = 0 - x_out
		fly.pub_twist.publish(fly.twist)
		
	print 'finished pitch'
	

	while pid_y_vel.cycle_count <3:
		y_out = pid_y_vel.tune(fly.drone[1])
		fly.twist.linear.z = 0
		fly.twist.linear.x = 0 - y_out
		fly.twist.linear.y = 0 
		fly.pub_twist.publish(fly.twist)
	print 'finished roll'


	# pass the values obtained to the drone_control class
	fly.kp[1] = pid_x_vel.kp
	fly.ki[1] = pid_x_vel.ki
	fly.kd[1] = pid_x_vel.kd

	fly.kp[0] = pid_y_vel.kp
	fly.ki[0] = pid_y_vel.ki
	fly.kd[0] = pid_y_vel.kd

	fly.kp[2] = pid_z_vel.kp
	fly.ki[2] = pid_z_vel.ki
	fly.kd[2] = pid_z_vel.kd

	

	print 'z_kp\t' + str(pid_z_vel.kp) + '\tz_ki\t' + str(pid_z_vel.ki) + '\tz_kd\t' + str(pid_z_vel.kd)
	print 'x_kp\t' + str(pid_x_vel.kp) + '\tz_ki\t' + str(pid_x_vel.ki) + '\tx_kd\t' + str(pid_x_vel.kd)
	print 'y_kp\t' + str(pid_y_vel.kp) + '\tz_ki\t' + str(pid_y_vel.ki) + '\ty_kd\t' + str(pid_y_vel.kd)
	print '--------------------------------------------------------------------------------------------------'
	print '--------------------------------------------auto-tuning complete----------------------------------'


	# carry out waypoint navigation
	while not rospy.is_shutdown():
		fly.pub_empty_takeoff.publish(Empty())
		fly.pid()

		# a wavepoint is traversed
		if fly.done:
			# re-initialise the flag
			fly.done = False

			# make the Iterm zero before changing the setpoint
			fly.Iterm = [0.0, 0.0, 0.0]
			
			# change the index of destination list
			fly.count = fly.count + 1

		# check if all points are traversed
		if fly.count == fly.length - 1:
			break

	# land the drone
	fly.pub_empty_landing.publish(Empty())

		






		










