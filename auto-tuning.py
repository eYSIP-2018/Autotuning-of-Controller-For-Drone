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
		self.dest = [0, 0, 20]    #defines the setpoint/target

		self.prev_error_x = 0.0  #to track previous errors
		self.prev_error_y = 0.0
		self.prev_error_z = 0.0

		self.drone = [0.0,0.0,0.0] # drone location

		# pitch parameters
		self.pitch_kp =None    
		self.pitch_ki =None
		self.pitch_kd =None

		#roll parameters
		self.roll_kp =None	
		self.roll_ki =None
		self.roll_kd =None

		#throttle parameters
		self.throttle_kp =None   
		self.throttle_ki = None
		self.throttle_kd = None

		# Max values of roll,pitch,throttle
		self.rc_roll_max =1575
		self.rc_pitch_max=1575
		self.rc_throttle_max=1800

		# Min values of roll,pitch,throttle
		self.rc_roll_min =1425
		self.rc_pitch_min = 1425
		self.rc_throttle_min =1200

		

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
		self.throttle_Iterm=0.0
		self.roll_Iterm= 0.0
		self.pitch_Iterm=0.0


		# sampling time
		self.sample_time = 0.030

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


	'''
	method : get_data()
	use : to determine the location of the drone
	'''

	def get_data(self,msg):
		self.drone[0] = msg.poses[0].position.x  # x coordinate of the drone
		self.drone[1] = msg.poses[0].position.y  # y coordinate of the drone
		self.drone[2] = msg.poses[0].position.z  # z coordinate of the drone



	def pid_calc(self):
		error_x = self.dest[0]- self.drone[0]  # difference between x coordinate of setpoint and drone
		error_y = self.dest[1]- self.drone[1]  # difference between y coordinate of setpoint and drone
		error_z = self.dest[2]- self.drone[2]  # difference between z coordinate of setpoint and drone
		curr_time = time.time()                # current time
		dt = curr_time - self.prev_time			# change in time

		if (dt< self.sample_time):   # if time gap is very small than the sampling time; return
			#print "dt" + str(dt) 
			return

		self.prev_time = curr_time	# update prev_time

		if(abs(error_z)<0.1 and abs(error_y)<0.1 and abs(error_z)<0.1): #(used for wavepoint navigation)
			return
		
		else:

			# updating Iterm to reduce steady state error

			# scaled version of error is added each time so as to make sure that the Iterm doesnt go very high within a second

			# scaling factors were first determined by cakculation, considering the value of Ki and then again varied depending upon the observation.
			self.roll_Iterm = self.roll_Iterm  + (error_y/20)*self.roll_ki
			self.pitch_Iterm = self.pitch_Iterm  + (error_x/20)*self.pitch_ki
			self.throttle_Iterm = self.throttle_Iterm  + (error_z/100)*self.throttle_ki

			self.pitch_err_pub.publish(error_x)
			self.roll_err_pub.publish(error_y)
			self.alt_err_pub.publish(error_z)


			# PID outputs
			
			self.out_roll = (self.roll_kp * error_y)  + self.roll_Iterm + (self.roll_kd * (error_y- self.prev_error_y))

			self.out_pitch = (self.pitch_kp * error_x) + self.pitch_Iterm + (self.pitch_kd *(error_x - self.prev_error_x))

			self.out_throttle= (self.throttle_kp * error_z) + self.throttle_Iterm + (self.throttle_kd * (error_z - self.prev_error_z))




			# System Outputs = equilibrium_value +/- pid output
			self.cmd.rcRoll= 1490  + self.out_roll
			self.cmd.rcPitch =1500 + self.out_pitch
			self.cmd.rcThrottle = 1500 - self.out_throttle

			# Capping the System outputs if required
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


			# Updating previous error
			self.prev_error_x = error_x
			self.prev_error_y = error_y
			self.prev_error_z = error_z
		
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
	pid_throttle = AutoTune(fly.dest[2], fly.sample_time, 300, -300, 3,True)

	# initialise the object pid_pitch to autotune the pitch params
	pid_pitch    = AutoTune(fly.dest[0], fly.sample_time, 75, -75 ,3,False)

	# initialise the object pid_rollh to autotune the roll params
	pid_roll     = AutoTune(fly.dest[1], fly.sample_time, 75, -75, 3,False)

	
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

	fly.pitch_kp = pid_pitch.kp
	fly.pitch_ki = pid_pitch.ki
	fly.pitch_kd = pid_pitch.kd



	fly.roll_kp = pid_roll.kp
	fly.roll_ki = pid_roll.ki
	fly.roll_kd = pid_roll.kd

	fly.throttle_kp = pid_throttle.kp
	fly.throttle_ki = pid_throttle.ki
	fly.throttle_kd = pid_throttle.kd



	print 'throttle_kp\t' + str(pid_throttle.kp) + '\tthrottle_ki\t' + str(pid_throttle.ki) + '\tthrottle_kd\t' + str(pid_throttle.kd)
	print 'pitch_kp\t' + str(pid_pitch.kp) + '\tpitch_ki\t' + str(pid_pitch.ki) + '\tpitch_kd\t' + str(pid_pitch.kd)
	print 'roll_kp\t' + str(pid_roll.kp) + '\troll_ki\t' + str(pid_roll.ki) + '\troll_kd\t' + str(pid_roll.kd)
	print '--------------------------------------------------------------------------------------------------'


	while not rospy.is_shutdown():
		fly.pid_calc()

    



	



	