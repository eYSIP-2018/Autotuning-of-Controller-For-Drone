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


class drone_control():
	def __init__(self):
		rospy.init_node('control_drone')


		self.done = False
		self.count =0
		self.done_counter = 0

		self.dest = [ (0, 0, 20) , (4,-4,20), (0,0,30)]    #defines the setpoint/target
		self.land = [ 0, 0, 30]

		self.prev_error = [0.0, 0.0, 0.0]  #to track previous 

		self.drone = [0.0,0.0,0.0] # drone location

		
		self.kp = [ None, None, None ]   
		self.ki = [ None, None, None ]
		self.kd = [ None, None, None ]

		
		self.rc_max = [1575, 1575, 1800]
		self.rc_min = [1425, 1425, 1200]

		

		self.cmd = PlutoMsg()
		self.cmd.rcRoll = 1500
		self.cmd.rcPitch = 1500
		self.cmd.rcYaw = 1500
		self.cmd.rcThrottle = 1500
		self.cmd.rcAUX1 = 1500
		self.cmd.rcAUX2 = 1500
		self.cmd.rcAUX3 = 1500
		self.cmd.rcAUX4 = 1000

		# Iterm Initialisation
		self.Iterm = [0.0, 0.0, 0.0]

		# sampling time
		self.sample_time = 0.025

		# defining previous time
		self.prev_time = time.time()

		

		# Initialising the /drone_command publisher
		self.command_pub = rospy.Publisher('/drone_command', PlutoMsg, queue_size=1)

		# Initialising subscriber to subscribe to the topic /whycon/poses to get current drone location
		rospy.Subscriber('/whycon/poses',PoseArray,self.get_data)
		
		# defining the following publishers to plot the error graph 
		self.roll_err_pub = rospy.Publisher('/roll_error', Float64, queue_size=1)
		self.pitch_err_pub = rospy.Publisher('/pitch_error', Float64, queue_size=1)
		self.alt_err_pub = rospy.Publisher('/alt_error', Float64, queue_size=1)
		
		# to arm the drone ( optional )
		self.arm()

	def arm(self):
		self.cmd.rcRoll = 1500
		self.cmd.rcYaw = 1500
		self.cmd.rcPitch = 1500
		self.cmd.rcThrottle = 1000
		self.cmd.rcAUX4 = 1500
		self.command_pub.publish(self.cmd)
		rospy.sleep(0.1)

	def disarm(self):
		self.cmd.rcThrottle =1300
		self.cmd.rcAUX4 = 1200
		self.command_pub.publish(self.cmd)
		rospy.sleep(1)


	'''
	method : get_data()
	use : to determine the location of the drone
	'''

	def get_data(self,msg):
		self.drone[0] = msg.poses[0].position.x  # x coordinate of the drone
		self.drone[1] = msg.poses[0].position.y  # y coordinate of the drone
		self.drone[2] = msg.poses[0].position.z  # z coordinate of the drone



	def pid_calc(self):
		error_x = self.dest[self.count][0]- self.drone[0]  # difference between x coordinate of setpoint and drone
		error_y = self.dest[self.count][1]- self.drone[1]  # difference between y coordinate of setpoint and drone
		error_z = self.dest[self.count][2]- self.drone[2]  # difference between z coordinate of setpoint and drone
		curr_time = time.time()                # current time
		dt = curr_time - self.prev_time			# change in time

		if (dt< self.sample_time):   # if time gap is very small than the sampling time; return
			#print "dt" + str(dt) 
			return

		self.prev_time = curr_time	# update prev_time

		if(abs(error_z)<0.5 and abs(error_y)<0.5 and abs(error_z)<0.7) :
			self.done_counter += 1
			print (self.done_counter)
			if self.done_counter > 100:
				self.done_counter = 0
				print (self.done_counter)
				self.done = True 
				return
		
		else:

			# updating Iterm to reduce steady state error

			# scaled version of error is added each time so as to make sure that the Iterm doesnt go very high within a second

			# scaling factors were first determined by cakculation, considering the value of Ki and then again varied depending upon the observation.
			self.Iterm[1] = self.Iterm[1]  + (error_y/20)*self.ki[1]
			self.Iterm[0] = self.Iterm[0]  + (error_x/20)*self.ki[0]
			self.Iterm[2] = self.Iterm[2]  + (error_z/100)*self.ki[2]

			self.pitch_err_pub.publish(error_x)
			self.roll_err_pub.publish(error_y)
			self.alt_err_pub.publish(error_z)


			# PID outputs
			
			self.out_roll = (self.kp[1] * error_y)  + self.Iterm[1] + (self.kd[1] * (error_y- self.prev_error[1]))

			self.out_pitch = (self.kp[0] * error_x) + self.Iterm[0] + (self.kd[0] *(error_x - self.prev_error[0]))

			self.out_throttle= (self.kp[2] * error_z) + self.Iterm[2] + (self.kd[2] * (error_z - self.prev_error[2]))




			# System Outputs = equilibrium_value +/- pid output
			self.cmd.rcRoll= 1490  + self.out_roll
			self.cmd.rcPitch =1500 + self.out_pitch
			self.cmd.rcThrottle = 1500 - self.out_throttle

			# Capping the System outputs if required
			if self.cmd.rcThrottle > self.rc_max[2]:
				self.cmd.rcThrottle=self.rc_max[2]
			elif self.cmd.rcThrottle < self.rc_min[2]:
				self.cmd.rcThrottle = self.rc_min[2]

			if self.cmd.rcRoll > self.rc_max[1]:
				self.cmd.rcRoll=self.rc_max[1]
			elif self.cmd.rcRoll < self.rc_min[1]:
				self.cmd.rcRoll = self.rc_min[1]

			if self.cmd.rcPitch > self.rc_max[0]:
				self.cmd.rcPitch = self.rc_max[0]
			elif self.cmd.rcPitch < self.rc_min[0]:
				self.cmd.rcPitch = self.rc_min[0]


			# Updating previous error
			self.prev_error[0] = error_x
			self.prev_error[1] = error_y
			self.prev_error[2] = error_z
		
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

				

				
				# Ziegler nichols formula for classical PID
				self.kp = 0.8 * self.ku
				self.ki = (self.kp / (0.5 * self.tu)) * self.sample_time
				self.kd = (0.125 * self.kp * self.tu) / self.sample_time
				



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
	pid_throttle = AutoTune(fly.dest[0][2], fly.sample_time, 300, -300, 3, True)

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
		fly.pid_calc()
		if fly.done :
			fly.done = False
			fly.Iterm = [0.0, 0.0, 0.0]
			fly.count +=1
			if fly.count >3:
				break

	fly.disarm()
	





    



	



	