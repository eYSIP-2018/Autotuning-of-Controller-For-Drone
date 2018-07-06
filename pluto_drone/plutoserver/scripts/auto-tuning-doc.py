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

		# points to navigate
		self.dest = [(0, 0, 20), (-4,-4,15), (0,0,30)]

		# length of dest list
		self.len = len(self.dest)

		self.prev_time = time.time()


		# initialise a variable that will inidcate if a waypoint is succesfully traversed
		self.done = False
		self.done_counter = 0

		# a counter that keeps track of waypoint traversed
		self.count = 0

		# create a list for Kp, Ki, Kd for motion along all three x, y and z axes
		self.kp = [None, None, None]
		self.ki = [None, None, None]
		self.kd = [None, None, None]

		# drone's current location
		self.drone = [0.0,0.0,0.0]

		# keep track of previous error in x, y, z coordinates
		self.prev_error_x = 0.0
		self.prev_error_y = 0.0
		self.prev_error_z = 0.0


		# max values of Pitch, Roll, Throttle
		self.rc_roll_max =1575
		self.rc_pitch_max=1575
		self.rc_throttle_max=1800

		# min values of Pitch, Roll, Throttle
		self.rc_roll_min =1425
		self.rc_pitch_min = 1425
		self.rc_throttle_min =1200

		# list to hold sum of errors
		self.sum_err_x =[]
		self.sum_err_y =[]
		self.sum_err_z =[]

		# initialise the Iterm to sum up errors in all three axes
		self.Iterm = [0.0, 0.0, 0.0]

		# drone_parameters
		self.cmd = PlutoMsg()
		self.cmd.rcRoll = 1500 
		self.cmd.rcPitch = 1500 
		self.cmd.rcYaw = 1000
		self.cmd.rcThrottle = 1000
		self.cmd.rcAUX1 = 1500
		self.cmd.rcAUX2 = 1500
		self.cmd.rcAUX3 = 1500
		self.cmd.rcAUX4 = 1000

		#initialise sample time
		self.sample_time = 0.030

		#initialise a timer that will be used in waypoint navigtaion
		self.t1= time.time()
		


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

	# method to get the drone location
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
		error_x = self.dest[self.count][0]- self.drone[0]

		# difference in y-coordinate of detsination and drone 
		error_y = self.dest[self.count][1]- self.drone[1]

		# difference in z-coordinate of detsination and drone 
		error_z = self.dest[self.count][2]- self.drone[2]

		# current time
		curr_time = time.time()

		# variable that holds the time lapse
		dt = curr_time - self.prev_time


		# in order to make sure if the drone was stable at a wyapoint for a considerable amount of time
		# do the following for every time gap of 0.3 seconds
		if (curr_time - self.t1 >0.3):

			# every 0.3 seconds store the error in each coordinates
			self.sum_err_x.append(error_x)
			self.sum_err_y.append(error_y)
			self.sum_err_z.append(error_z)
			
			# In one second the length of the above list becomes 3, hence after every second delete the first element(oldest error)
			if len(self.sum_err_x) > 3:
				self.sum_err_x.pop(0)

			# sum up the absolute value of elemnts in list
			sum_x = sum(map(abs,self.sum_err_x))

			# same as above
			if len(self.sum_err_y) > 3:
				self.sum_err_y.pop(0)
			sum_y = sum(map(abs,self.sum_err_y))

			if len(self.sum_err_z) > 3:
				self.sum_err_z.pop(0)
			sum_z = sum(map(abs,self.sum_err_z))

			# update the timer
			self.t1 = curr_time

			# check if the sum is less than the permited error range
			if sum_x<=2 and sum_y <=2 and sum_z<=3:

				# update the counter,
				self.done_counter += 1
				print self.done_counter

				# checks if the drone was stable for 3 * 5 seconds.
				if self.done_counter >7:

					# if staisfied
					self.done_counter = 0
					self.done = True 
					return

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

			# updating Iterm to reduce steady state error
			# scaled version of error is added each time so as to make sure that the Iterm doesnt go very high within a second
			# scaling factors were first determined by cakculation, considering the value of Ki and then again varied depending upon the observation.
			self.Iterm[1] = self.Iterm[1]  + (error_y/20)
			self.Iterm[0] = self.Iterm[1]  + (error_x/20)
			self.Iterm[2] = self.Iterm[2]  + (error_z/100)
		
			# determine the pid output on roll, pitch, throttle axes
			self.out_roll = (self.kp[1] * error_y) + (self.Iterm[1] * self.ki[1]) + (self.kd[1] * (error_y- self.prev_error_y))
			self.out_pitch = (self.kp[0] * error_x) + (self.Iterm[0] * self.ki[0]) + (self.kd[0] *(error_x - self.prev_error_x))
			self.out_throttle= (self.kp[2] * error_z) + (self.Iterm[2] * self.ki[2]) + (self.kd[2] * (error_z - self.prev_error_z))

			
			
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



			# update previous error
			self.prev_error_x = error_x
			self.prev_error_y = error_y
			self.prev_error_z = error_z
		
			# publish the computesd values
			self.command_pub.publish(self.cmd)

# Class AutoTune used to define the structure of autotuning
# input arguments : setpoint --> the coordinate around which oscillations will occur
#					sample_time --> same sample_time used in PID logic ; used to determine KI and KD here
#					max_val  --> maximum output value of the PID
# 					min_val -->  minimum output value of the PID
#					tuning_cycles --> number of oscilattions to be performed
#					z_flag --> indicate if this class is called to tune the params of throttle (to copensate for the z coordinate variaton(explained below))


class AutoTune():

	def __init__(self, setpoint, sample_time, max_val, min_val, tunning_cycles ,z_flag):
		self.setpoint = setpoint
		self.sample_time = sample_time
		self.max_out = max_val
		self.min_out = min_val
		self.tunning_cycles = tunning_cycles
		self.positive_cycle = True   # initially set this flag to True (assume the oscillations start from positive half)
		self.out = 0
		self.prev_time =time.time()
		self.flag = z_flag

# initialisation of the tuning loop
	def startTunningLoop(self):
		self.cycle_count = 0 		#counter for number of oscillations
		self.out = self.max_out		#initialise output of this method(which will be published to /drone_command to maximum value)

		self.t1 = self.t2 = time.time() #initialise timers
		self.tLow = self.tHigh = 0		# time periods of positive and negative cycles of oscillation

		self.max = -10		# initialise max possible value of coordinate to a minimum value initially
		self.min = 10		# initialise min possible value of coordinate to a maximum value initially
		#These values will be modified ahead

		#initialise avg values of kp, ki, kd
		self.p_avg = self.i_avg = self.d_avg = 0

	def tune(self, input):
		self.prev_time = time.time()

		# gives the maximum of the input i.e the current coordinate of drone and self.max
		self.max = max(self.max, input)

		# gives the minimum of the input i.e the current coordinate of drone and self.min
		self.min = min(self.min, input)

		# current coordinate is greater than setpoint coordinate
		if self.positive_cycle:

			# check z_flag ; If z_flag is set then it means throttle autotuning is in progress
			# as the z_coordinate changes by +/- 1.5(max) consider this offset.
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

			# if z_flag is false it means the pitch or roll autotuning is in progress hence there is no need of offset
			if not self.flag and input > self.setpoint:

				#logic remains same as above
				self.positive_cycle = False
				self.out = self.min_out
				self.t1 = time.time()
				self.tHigh = self.t1 - self.t2
		
		# if oscilation has entered negative cycle 
		if not self.positive_cycle:

			# check z_flag ; If z_flag is set then it means throttle autotuning is in progress
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
				self.tu = self.tLow + self.tHigh

				

				'''
				# Ziegler nichols formula for classical PID
				self.kp = 0.8 * self.ku
				self.ki = (self.kp / (0.5 * self.tu)) * self.sample_time
				self.kd = (0.125 * self.kp * self.tu) / self.sample_time
				'''

				
				# Ziegler nichols formula for Pessen Integral Rule
				self.kp = 0.7 * self.ku
				self.ki = (self.kp / (0.4 * self.tu)) * self.sample_time
				self.kd = (0.15 * self.kp * self.tu) / self.sample_time
				
				
				'''
				# Ziegler nichols formula for some overshoot
				self.kp = 0.33 * self.ku
				self.ki = (self.kp / (0.5 * self.tu)) * self.sample_time
				self.kd = (0.33 * self.kp * self.tu) / self.sample_time
				'''

				'''
				# Ziegler nichols formula for no overshoot
				self.kp = 0.2 * self.ku
				self.ki = (self.kp / (0.5 * self.tu)) * self.sample_time
				self.kd = (0.33 * self.kp * self.tu) / self.sample_time
				'''

				if self.cycle_count:

					# to detrmine the average of the values over the total nimber of ocsillations
					self.p_avg += self.kp 	
					self.i_avg += self.ki
					self.d_avg += self.kd

				# after the first oscillation initialise the min value to setpoint 
				self.min = self.setpoint

				# increment the cycle counter
				self.cycle_count= self.cycle_count +1
				print self.cycle_count


			# same logic as above ; for roll and pitch ; z_flag =false
			if not self.flag and input < self.setpoint:
				self.positive_cycle = True
				self.out = self.max_out
				self.t2 = time.time()
				self.tLow = self.t2 - self.t1
				self.ku = (4.0 * ((self.max_out - self.min_out) / 2.0)) / (3.142 * (self.max - self.min) / 2.0)
				self.tu = self.tLow+self.tHigh
				self.kp = 0.8 * self.ku
				#print self.kp
				self.ki = (self.kp / (0.5 * self.tu)) * self.sample_time
				self.kd = (0.25 * self.kp * self.tu) / self.sample_time

				if self.cycle_count:
					self.p_avg += self.kp
					self.i_avg += self.ki
					self.d_avg += self.kd

				self.min = self.setpoint
				self.cycle_count= self.cycle_count +1
				print self.cycle_count

		if self.cycle_count >= self.tunning_cycles:
			self.positive_cycle = False
			self.out = self.min_out

			self.kp = self.p_avg/(self.cycle_count-1)
			self.ki = self.i_avg/(self.cycle_count-1)
			self.kd = self.d_avg/(self.cycle_count-1)


		#print self.out

		# return the output that will be published to /drone_command
		return self.out
		
if __name__ == '__main__':

	fly= drone_control()

	# initialise the object pid_throttle to autotune the throttle params
	pid_throttle = AutoTune(fly.dest[0][2], fly.sample_time, 200, -200, 3, True)

	# initialise the object pid_pitch to autotune the pitch params
	pid_pitch    = AutoTune(fly.dest[0][0], fly.sample_time, 75, -75 ,3,False)

	# initialise the object pid_rollh to autotune the roll params
	pid_roll     = AutoTune(fly.dest[0][1], fly.sample_time, 75, -75, 3,False)

	
	pid_throttle.startTunningLoop()
	pid_pitch.startTunningLoop()
	pid_roll.startTunningLoop()


	# to carry out oscillations thehreby determining the KP , KI and KD vaues of throttle,pitch and roll
	while pid_throttle.cycle_count <3:
		throttle_out = pid_throttle.tune(fly.drone[2])
		fly.cmd.rcThrottle = 1500 - throttle_out
		fly.cmd.rcRoll= 1500
		fly.cmd.rcPitch =1500
		fly.command_pub.publish(fly.cmd)
	print 'finished throttle'
	rospy.sleep(2)
	


	while pid_pitch.cycle_count <3:
		pitch_out = pid_pitch.tune(fly.drone[0])
		fly.cmd.rcPitch =1500 + pitch_out
		fly.cmd.rcRoll= 1500
		fly.cmd.rcThrottle =1550
		fly.command_pub.publish(fly.cmd)
	print 'finished pitch'
	rospy.sleep(2)

	while pid_roll.cycle_count <3:
		roll_out = pid_roll.tune(fly.drone[1])
		fly.cmd.rcRoll =1500 + roll_out
		fly.cmd.rcPitch= 1500
		fly.cmd.rcThrottle =1550
		fly.command_pub.publish(fly.cmd)
	print 'finished roll'


	# pass the values obtained to the drone_control class

	fly.kp[0] = pid_pitch.kp
	fly.ki[0] = pid_pitch.ki
	fly.kd[0] = pid_pitch.kd



	fly.kp[1] = pid_roll.kp
	fly.ki[1] = pid_roll.ki
	fly.kd[1] = pid_roll.kd

	fly.kp[2] = pid_throttle.kp
	fly.ki[2] = pid_throttle.ki
	fly.kd[2] = pid_throttle.kd


	print 'throttle_kp\t' + str(pid_throttle.kp) + '\tthrottle_ki\t' + str(pid_throttle.ki) + '\tthrottle_kd\t' + str(pid_throttle.kd)
	print 'pitch_kp\t' + str(pid_pitch.kp) + '\tpitch_ki\t' + str(pid_pitch.ki) + '\tpitch_kd\t' + str(pid_pitch.kd)
	print 'roll_kp\t' + str(pid_roll.kp) + '\troll_ki\t' + str(pid_roll.ki) + '\troll_kd\t' + str(pid_roll.kd)
	print '--------------------------------------------------------------------------------------------------'

	
	while not rospy.is_shutdown():
		fly.pid()
	
		if fly.done :
			fly.done = False
			fly.Iterm = [0.0, 0.0, 0.0]
			fly.count +=1
			
			if fly.count > fly.len -1 :
				break
	print 'disarm'
	fly.disarm()
	






    



	



	
