#!/usr/bin/env python
from plutodrone.srv import *
from std_msgs.msg import Int64
from std_msgs.msg import Float64
import rospy
import math
class request_data():
	def __init__(self):
		rospy.init_node('drone_board_data')
		print 'rotate the drone to see the graph'
		data = rospy.Service('PlutoService', PlutoPilot, self.access_data)
		self.mag = rospy.Publisher('/magnetometer', Float64, queue_size=1)	
		rospy.spin()

	def access_data(self, req):

		if req.magX != 0:
			mag_rad = math.atan2(req.magY , req.magX)
			self.mag.publish(mag_rad)			

		rospy.sleep(.1)
		return PlutoPilotResponse(rcAUX2 =1500)
		 
test = request_data()
		
