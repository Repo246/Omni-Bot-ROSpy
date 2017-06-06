#
#	move_to_xy.py - Moves the robot vectorally to the specified coordinates by 
# 		building a movement message, for the Arduino Motor controller, from current
#		position and heading data. Target coordinates are received from the 
#		'clicked_point' topic published by rViz.  
#	written by: T.Reese
#	
# 	ROS ref: http://wiki.ros.org and http://wiki.ros.org/rviz
#
# 	Required nodes: Robot_Localization, BNO055, Encoders 


import rospy
import sys
import tf
import serial
import struct
from math import pi
from math import atan2
from math import pow
from math import sqrt
from locale import atof
from time import sleep
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped


class move_to_xy():
	def __init__(self):
		# Initialize 'motor_updater' node
		rospy.init_node('move_xy')
		
		# Initialize subscription to '/odometry/filtered' topic 
		rospy.Subscriber('/odometry/filtered', Odometry, self.filter_callback, queue_size = 100)
		
		# Initialize subscription to 'clicked_point' topic 
		rospy.Subscriber('clicked_point', PointStamped, self.click2pt_callback, queue_size = 100)
		
		# Initialize publisher to the 'move_status' topic  
		self.movepub = rospy.Publisher('move_status', String, queue_size = 10)


		# Specify and initialize the port Motor Arduino is on 
		try:
			self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout = 1, write_timeout = 1)
		except: 
			self.ser = serial.Serial('/dev/ttyUSB1', 115200, timeout = 1, write_timeout = 1)
			
#		self.f = open('data.txt', 'w')		# Data capture file
		
		# Incoming message variables
		self.posX = 0.0
		self.posY = 0.0
		self.curr_heading = 0
		self.prevX = 0.0	
		self.prevY = 0.0
		self.new_target = False
		self.new_pose = False
		self.target = [0.0,0.0]
		self.Listener = tf.TransformListener()
		self.Listener.waitForTransform('/odom','/map',rospy.Time(),rospy.Duration(5.0))
		
		# Outgoing message variables
		self.move_done = "Move Complete"
		
		# Movement variaables
		self.curr_movement = 'stopped'
		self.start_PWM = 110
		self.curr_PWM = self.start_PWM
		self.start_PPS = 600
		self.setPoint = 0.0
		self.clockwise = 1
		self.cntr_clck = -1	
		self.dir_to_turn = 1
		self.buff = bytes()
		
		# Coordinate variables									
		self.target_dist = 0.0
		self.target_heading = 0.0
		self.rel_x = 0.0					
		self.rel_y = 0.0
		self.dist2prev = 0.0
		
		# Ramp down variables
		self.ramp_strt = 0.2
		self.ramp_dist = self.ramp_strt
		self.pwm_portion = int(0.05*self.start_PWM)
		
		
		while self.new_pose == False and not rospy.is_shutdown():
			sleep(0.01)
		
		while not rospy.is_shutdown():
			if self.new_target == True:
				self.new_target = False
				self.travel()
			else: 
				sleep(0.01)
#		self.f.close()				# Close data capture file
	
	#--------------------------------------------------------------------------------
	# travel() - Moves the robot to given coordinates. Publishes "Move Complete"  
	# 	string message when travel is done.  
	#--------------------------------------------------------------------------------		
	def travel(self):
		print "Calculating target heading and distance"
		#------Adjust heading, if necessary, for travel to target----------
		self.rotate()
		#--------------------Advance to target-----------------------------
		self.advance()		
		print "'Movement complete' message sent"
		self.movepub.publish(self.move_done)
	
	#--------------------------------------------------------------------------------
	# filter_callback() - Updates the position and heading variables for this class.  
	#--------------------------------------------------------------------------------		
	def filter_callback(self, data):
		self.posX = data.pose.pose.position.x
		self.posY = data.pose.pose.position.y
		# Compute the angle relative to the starting angle
		Q =  tf.transformations.euler_from_quaternion([data.pose.pose.orientation.x, \
			data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w])
		self.curr_heading = Q[2]
		self.new_pose = True
		
	#--------------------------------------------------------------------------------
	# click2pt_callback() - Sets the target coordinates to move toward.  
	#--------------------------------------------------------------------------------	
	def click2pt_callback(self, data):
		newpoint = self.Listener.transformPoint('/odom',data)
		self.target[0] = newpoint.point.x
		self.target[1] = newpoint.point.y
		self.new_target = True

	#--------------------------------------------------------------------------------
	# calc_vector() - Calculates the distance and heading to the target  
	# 	coordinates. 
	#--------------------------------------------------------------------------------
	def calc_vector(self):							
		self.rel_x = self.target[0] - self.posX					# x coordinate relative to current position
		self.rel_y = self.target[1] - self.posY					# y coordinate relative to current position
		a_sqrd = pow(self.rel_x, 2)								# x component of the vector to target, squared 
		b_sqrd = pow(self.rel_y, 2)								# y component of the vector to target, squared
		self.target_dist = sqrt(a_sqrd + b_sqrd) 				# Pythagorean theorem to find distance to target
		self.target_heading = atan2(self.rel_y,self.rel_x)		# arctan of relative x and relative y to find target direction
		
		self.rel_x = self.prevX - self.posX						# x coordinate of last position relative to current position
		self.rel_y = self.prevY - self.posY						# y coordinate of last position relative to current position
		a_sqrd = pow(self.rel_x, 2)								# x component of the vector to last position, squared 
		b_sqrd = pow(self.rel_y, 2)								# y component of the vector to last position, squared
		self.dist2prev = sqrt(a_sqrd + b_sqrd) 					# Pythagorean theorem to find distance to last position
		
	#--------------------------------------------------------------------------------
	# send_message() - Based on the type of action, function builds message with  
	# 	calculated parameters and sends it. 
	#--------------------------------------------------------------------------------
	def send_message(self, msg_type):
		#-----Add message keys to beginning of Arduino messages----- 
		self.buff = bytes()
		msg_key1 = 123
		self.buff = struct.pack('B',msg_key1)
		msg_key2 = 132
		self.buff += struct.pack('B',msg_key2)
		msg_key3 = 231
		self.buff += struct.pack('B',msg_key3)
		
		if msg_type == "advance":								# Build and send the 'Advance' message
			stmsg_ID = 17
			self.buff += struct.pack('B',stmsg_ID)
			# Break the current heading up into 4 bytes-----
			self.buff += struct.pack('f',self.curr_heading)			
			# Break the target heading up into 4 bytes------
			self.buff += struct.pack('f',self.target_heading)
			# Break the current PWM into bytes
			self.buff += struct.pack('B',self.curr_PWM)		
			self.ser.write(self.buff)							
			self.ser.flush()									
			
		elif msg_type == "ramp down":							# Build and send the 'Ramp down' message
			stmsg_ID = 17
			self.buff += struct.pack('B',stmsg_ID)
			# Break the current heading up into 4 bytes-----
			self.buff += struct.pack('f',self.curr_heading)			
			# Break the target heading up into 4 bytes------
			self.buff += struct.pack('f',self.setPoint)
#			self.buff += struct.pack('f',self.target_heading)	# Used to adjust the setpoint to the target as it ramps down
			# Break the current PWM into bytes
			self.buff += struct.pack('B',self.curr_PWM)	
			self.ser.write(self.buff)							
			self.ser.flush()									

		elif msg_type == "rotate":								# Build and send the 'Rotate' message
			stmsg_ID = 18
			self.buff += struct.pack('B',stmsg_ID)
			# Break the current heading up into 4 bytes-----
			self.buff += struct.pack('f',self.curr_heading)	
			# Break the setpoint PPS into 2 bytes-----------
			setPPS = self.start_PPS*self.dir_to_turn
			self.buff += struct.pack('h',setPPS)
			self.ser.write(self.buff)							
			self.ser.flush()									

		elif msg_type == "stop":								# Build and send the 'Stop' message
			stmsg_ID = 19
			self.buff += struct.pack('B',stmsg_ID)
			self.ser.write(self.buff)							
			self.ser.flush()									

		elif msg_type == "heading":								# Build and send the 'Heading' message
			stmsg_ID = 20
			self.buff += struct.pack('B',stmsg_ID)
			# Break the current heading up into 4 bytes-----
			self.buff += struct.pack('f',self.curr_heading)		
			self.ser.write(self.buff)							
			self.ser.flush()										
	
	#--------------------------------------------------------------------------------
	# advance() - Sends a message to move forward to the Arduino. Ramps down PWM
	# 	when the distance remaining to the target is within a certain range.
	# 
	# Thought: it may be useful to resend the target heading(as a new setpoint) 
	#	to the Arduino PID, part way through the move, to reduce some of the 
	#	movement perpendicular to the direction of travel. 	         
	#--------------------------------------------------------------------------------
	def advance(self):
		self.prevX = self.posX									# Save the current X position
		self.prevY = self.posY									# Save the current Y position
		self.calc_vector()										# Calculate the current distance and heading to target
		self.ramp_dist = self.ramp_strt							# Prime local variable with distance to begin ramping PWM down
		self.setPoint = self.target_heading						# Save the initial setpoint heading for use during ramp down
		dist2travel = (self.target_dist - 0.02)					# Set distance to travel at the calculated distance to target
		self.dist2prev = 0.0
		prev_head = 0.0
		prev_dist = 0.0
		stalls = 0
		
		self.send_message("advance")							# send the 'Advance' message
		print "Advancing to target"
		
		while (self.target_dist > 0.01) and (self.dist2prev < dist2travel) and self.new_target == False and not rospy.is_shutdown(): 
			#----Ramp down PWM when 'ramp_dist' from target----- 
			if self.target_dist <= self.ramp_dist and self.curr_PWM > 70.0:
				self.ramp_dist = self.ramp_dist - 0.01
				self.curr_PWM = (self.curr_PWM - self.pwm_portion)
				self.send_message("ramp down")					# send the 'Ramp down' message
			#---------------------------------------------------	
			if prev_head != self.curr_heading:	
				self.send_message("heading")					# send the 'Heading' message						
				prev_head = self.curr_heading
			self.calc_vector()									# Calculate the current distance and heading to target
			#----Corrects a stalled advance by breaking loop----
			if prev_dist != self.target_dist:
				prev_dist = self.target_dist
				stalls = 0
			else:
				stalls+=1
				if stalls == 100:
					stalls = 0
					break
			#---------------------------------------------------
			#sleep(0.001)
		
		self.send_message("stop")								# send the 'Stop' message
		
		self.curr_PWM = self.start_PWM 							# Reset current linear PWM back to default
		sleep(1)
		while self.target_dist > 0.05:							# If the target is further than 5cm away move again
			self.travel()
		print "Advancing complete"
		

	#--------------------------------------------------------------------------------
	# rotate() - Determines the shortest direction of rotation to get to target 
	# 	heading. Sends the message to rotate to the Arduino. 
	#--------------------------------------------------------------------------------	
	def rotate(self):
		self.calc_vector()											# Calculate the current distance and heading to target
		if (abs(self.target_heading - self.curr_heading) > 0.01):
			print "Rotating to target heading"
			# Calculate the values needed for the rotation message 
			if self.target_heading > self.curr_heading:				# if target heading is larger than current heading
				turn_dist = self.target_heading - self.curr_heading	# --> turn_dist = counter-clockwise turn distance  
				if turn_dist <= pi:									# --> if turn_dist is less than pi 
					self.dir_to_turn = self.cntr_clck				# ----> turn counter-clockwise
				else:										
					self.dir_to_turn = self.clockwise				# else: turn clockwise
			else:													# if target heading is smaller than current heading
				turn_dist = self.curr_heading - self.target_heading	# --> turn_dist = clockwise turn distance
				if turn_dist <= pi:									# --> if turn_dist is less than pi
					self.dir_to_turn = self.clockwise				# ----> turn clockwise
				else:										
					self.dir_to_turn = self.cntr_clck				# else: turn counter-clockwise

			# send and send the Rotation message
			self.send_message("rotate")								# send the 'Rotation' message

			while not ((self.target_heading - 0.1) < self.curr_heading < (self.target_heading + 0.1)) and self.new_target == False and not rospy.is_shutdown():		
				sleep(0.0001) 
				
			self.send_message("stop")								# send the 'Stop' message
			sleep(1)
			if not ((self.target_heading - 0.1) < self.curr_heading < (self.target_heading + 0.1)):
				self.rotate()
			print "Rotation complete"
		else:
			print "Current heading is within acceptable range for travel"

		
if __name__ == "__main__" :
	move_to_xy()



