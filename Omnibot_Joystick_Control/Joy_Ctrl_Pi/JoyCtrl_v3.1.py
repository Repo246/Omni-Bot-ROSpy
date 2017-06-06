#
#	for_Pi_working - Receives controller commands, formulates a motor instruction, 
#					 then sends it to the Arduino Duemilanove via serial port. 
#
#		by Tyler Reese
#	
# References: 
#	PySerial - http://pyserial.readthedocs.io/en/latest/pyserial_api.html - Serial in python


import sys
import serial
import rospy
import time
from sensor_msgs.msg import Joy
from time import sleep



class ROS_Head:
	def __init__(self):
		# Initialize the node 'RasPi' 
		rospy.init_node('RasPi')

		# Specify and initialize the COM port Arduino is on 
		try:
			self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1, write_timeout=1)
		except: 
			self.ser = serial.Serial('/dev/ttyUSB1', 115200, timeout=1, write_timeout=1)  

		# new_state is a list formatted to be an instruction for the Arduino to execute.
		# The list is cast to a bytearray, after being altered by an event, and sent over 
		# the serial port to the Arduino event buffer to be executed in scheduled time.
		# Its format is: [start char,M1PWM,M2PWM,M3PWM,M1Dir,M2Dir,M3Dir,action,time(3),end char] 
		self.new_state = [17,0,0,0,1,1,1,0,0,0,0,19]
	
		self.curr_PWMs = [100, 80]							# List to hold current linear and rotational PWM values [lin,rot]
		self.hat = [0,0]
		self.old_axes = [0.0, 0.0]
		self.old_buttons = [0,0,0,0,0,0,0,0,0,0,0,0]
		self.msgQueued = False
		self.joy = Joy()
		self.num_btns = 12
		rospy.Subscriber('joy', Joy, self.joy_callback)

#		self.timer = 0										#-----------------DEBUG-----------------
		
	
		while not rospy.is_shutdown(): 						# While the exit button(#10) hasn't been pressed, continue
			if self.msgQueued:
				self.ser.write(bytearray(self.new_state))	# Sends new robot instruction over serial to Arduino
				#self.timer = time.clock() - self.timer		#-----------------DEBUG-----------------
				self.ser.flush()
				self.msgQueued = False
#				sys.stdout.write(str(self.timer) + ",")		#-----------------DEBUG-----------------
#				sys.stdout.flush()							#-----------------DEBUG-----------------
			sleep(0.01)


	# ------------------- Function Definitions --------------------		

	# Format for new_state --> [start char,M1PWM,M2PWM,M3PWM,M1Dir,M2Dir,M3Dir,action,time(3),end char]
	def newEvent(self):
		x = self.hat[0]
		y = self.hat[1]
		if x == 0:
			if y == 0:	    					# ROBOT REST:
				self.new_state[0] = 17			
				self.new_state[1] = 0					# Motor 1 --> speed = 0
				self.new_state[2] = 0					# Motor 2 --> speed = 0
				self.new_state[3] = 0					# Motor 3 --> speed = 0
				self.new_state[7] = 0					# Action: rest
				#	slowtoStop() ---> future function to implement for smooth stopping
			elif y == 1:    					# MOVE FORWARD(North):
				self.new_state[0] = 17
				self.new_state[1] = 0					# Motor 1 --> speed = 0
				self.new_state[2] = self.curr_PWMs[0]	# Motor 2 --> speed = current linear speed setting
				self.new_state[3] = self.curr_PWMs[0]	# Motor 3 --> speed = current linear speed setting
				self.new_state[5] = 0					# Motor 2 --> direction = clockwise
				self.new_state[6] = 1					# Motor 3 --> direction = counter-clockwise
				self.new_state[7] = 1					# Action: advance
			else:		    					# MOVE BACKWARD(South):
				self.new_state[0] = 17
				self.new_state[1] = 0
				self.new_state[2] = self.curr_PWMs[0]
				self.new_state[3] = self.curr_PWMs[0]
				self.new_state[5] = 1
				self.new_state[6] = 0
				self.new_state[7] = 5					# Action: retreat
		elif x == 1:
			if y == 0:							# WEST MOVEMENT:
				self.new_state[0] = 17
				self.new_state[1] = self.curr_PWMs[0]
				self.new_state[2] = int(self.curr_PWMs[0]*0.70)
				self.new_state[3] = int(self.curr_PWMs[0]*0.70)
				self.new_state[4] = 1
				self.new_state[5] = 0
				self.new_state[6] = 0
				self.new_state[7] = 7					# Action: head left
			elif y == 1:						# NORTHWEST MOVEMENT:
				self.new_state[0] = 17
				self.new_state[1] = self.curr_PWMs[0]	# Motor 1 --> speed = current linear speed setting
				self.new_state[2] = self.curr_PWMs[0]	# Motor 2 --> speed = current linear speed setting
				self.new_state[3] = 0					# Motor 3 --> speed = 0
				self.new_state[4] = 1					# Motor 1 --> direction = counter-clockwise
				self.new_state[5] = 0					# Motor 2 --> direction = clockwise
				self.new_state[7] = 8					# Action: head advance/left
			else:								# SOUTHWEST MOVEMENT:
				self.new_state[0] = 17
				self.new_state[1] = self.curr_PWMs[0]	# Motor 1 --> speed = current linear speed setting
				self.new_state[2] = 0					# Motor 2 --> speed = 0
				self.new_state[3] = self.curr_PWMs[0]	# Motor 3 --> speed = current linear speed setting
				self.new_state[4] = 1					# Motor 1 --> direction = counter-clockwise
				self.new_state[6] = 0					# Motor 3 --> direction = clockwise
				self.new_state[7] = 6					# Action: head retreat/left 
		else:
			if y == 0:							# EAST MOVEMENT:
				self.new_state[0] = 17
				self.new_state[1] = self.curr_PWMs[0]
				self.new_state[2] = int(self.curr_PWMs[0]*0.70)
				self.new_state[3] = int(self.curr_PWMs[0]*0.70)
				self.new_state[4] = 0
				self.new_state[5] = 1
				self.new_state[6] = 1
				self.new_state[7] = 3					# Action: head right
			elif y == 1:						# NORTHEAST MOVEMENT:
				self.new_state[0] = 17
				self.new_state[1] = self.curr_PWMs[0]	# Motor 1 --> speed = current linear speed setting
				self.new_state[2] = 0					# Motor 2 --> speed = 0
				self.new_state[3] = self.curr_PWMs[0]	# Motor 3 --> speed = current linear speed setting
				self.new_state[4] = 0					# Motor 1 --> direction = clockwise
				self.new_state[6] = 1					# Motor 3 --> direction = counter-clockwise
				self.new_state[7] = 2					# Action: head advance/right
			else:								# SOUTHEAST MOVEMENT:
				self.new_state[0] = 17
				self.new_state[1] = self.curr_PWMs[0]	# Motor 1 --> speed = current linear speed setting
				self.new_state[2] = self.curr_PWMs[0]	# Motor 2 --> speed = current linear speed setting
				self.new_state[3] = 0					# Motor 3 --> speed = 0
				self.new_state[4] = 0					# Motor 1 --> direction = clockwise
				self.new_state[5] = 1					# Motor 2 --> direction = counter-clockwise
				self.new_state[7] = 4					# Action: head retreat/right
		
	def buttonPressed(self, button):
		if (button == 2): 						# If button #3 is pressed, reduce linear_PWM 
			if (self.curr_PWMs[0] >= 20):
				self.curr_PWMs[0] = self.curr_PWMs[0] - 20
			else:
				self.curr_PWMs[0] = 0
			self.update_state()							# Update state with new PWM values
		elif (button == 3):						# If button #4 is pressed, increase linear_PWM	
			if (self.curr_PWMs[0] < 235):
				self.curr_PWMs[0] = self.curr_PWMs[0] + 20
			else:
				self.curr_PWMs[0] = 255
			self.update_state()							# Update state with new PWM values
		elif (button == 4):						# If button #5 is pressed, rotate counter-clockwise
			self.new_state[0] = 17
			self.new_state[1] = self.curr_PWMs[1]
			self.new_state[2] = self.curr_PWMs[1]
			self.new_state[3] = self.curr_PWMs[1]
			self.new_state[4] = 0
			self.new_state[5] = 0
			self.new_state[6] = 0
			self.new_state[7] = 9						# Action: "rotate counter-clockwise"
		elif (button == 5): 					# If button #6 is pressed, rotate clockwise
			self.new_state[0] = 17
			self.new_state[1] = self.curr_PWMs[1]
			self.new_state[2] = self.curr_PWMs[1]
			self.new_state[3] = self.curr_PWMs[1]
			self.new_state[4] = 1
			self.new_state[5] = 1
			self.new_state[6] = 1
			self.new_state[7] = 10						# Action: "rotate clockwise"	
		elif (button == 6):						# If button #7 is pressed, adjust tail right 
			self.new_state[0] = 18
			self.new_state[1] = 55
			self.new_state[2] = self.curr_PWMs[1]
			self.new_state[3] = self.curr_PWMs[1]
			self.new_state[4] = 0
			self.new_state[5] = 0
			self.new_state[6] = 0
			self.new_state[7] = 11
		elif (button == 7):						# If button #8 is pressed, adjust tail left 
			self.new_state[0] = 18
			self.new_state[1] = 55
			self.new_state[2] = self.curr_PWMs[1]
			self.new_state[3] = self.curr_PWMs[1]
			self.new_state[4] = 1
			self.new_state[5] = 0
			self.new_state[6] = 0
			self.new_state[7] = 12			

	def buttonReleased(self, button):
		if (button == 4):						# If button #5 is released, stop rotating 
			self.new_state[0] = 17
			self.new_state[1] = 0
			self.new_state[2] = 0
			self.new_state[3] = 0
			self.new_state[4] = 0
			self.new_state[5] = 0
			self.new_state[6] = 0
			self.new_state[7] = 0						# Action: rest
		elif (button == 5): 					# If button #6 is released, stop rotating
			self.new_state[0] = 17
			self.new_state[1] = 0
			self.new_state[2] = 0
			self.new_state[3] = 0
			self.new_state[4] = 1
			self.new_state[5] = 1
			self.new_state[6] = 1
			self.new_state[7] = 0						# Action: rest
		elif (button == 6):						# If button #7 is released, stop adjust tail right 
			self.new_state[0] = 18
			self.new_state[1] = 0
			self.new_state[2] = 0
			self.new_state[3] = 0
			self.new_state[4] = 0
			self.new_state[5] = 0
			self.new_state[6] = 0
			self.new_state[7] = 11
		elif (button == 7):						# If button #8 is released, stop adjust tail left 
			self.new_state[0] = 18
			self.new_state[1] = 0
			self.new_state[2] = 0
			self.new_state[3] = 0
			self.new_state[4] = 0
			self.new_state[5] = 0
			self.new_state[6] = 0
			self.new_state[7] = 12

	def update_state(self):
		if (self.new_state[1] > 0):
			self.new_state[1] = self.curr_PWMs[0]
		if (self.new_state[2] > 0):	
			self.new_state[2] = self.curr_PWMs[0]
		if (self.new_state[3] > 0):	
			self.new_state[3] = self.curr_PWMs[0]

	# Receive joystick data, formulate message. Forward new message to the motors 
	def joy_callback(self, data):
#		self.timer = time.clock()				#-----------------DEBUG(signal time to arduino)-----------------
		self.msgQueued = True
		left_right = data.axes[4]
		up_down = data.axes[5]
		if self.old_axes[0] == left_right and self.old_axes[1] == up_down:		
			for i in range(self.num_btns):
				if data.buttons[i] > self.old_buttons[i]:
					self.buttonPressed(i)
				elif data.buttons[i] < self.old_buttons[i]:
					self.buttonReleased(i) 
			self.old_buttons = data.buttons
		else:
			if left_right == 0.0:
				self.hat[0] = 0
			elif left_right == 1.0:
				self.hat[0] = 1
			else:
				self.hat[0] = -1
			if up_down == 0.0:
				self.hat[1] = 0
			elif up_down == 1.0:
				self.hat[1] = 1
			else:
				self.hat[1] = -1	
			self.old_axes[0] = left_right
			self.old_axes[1] = up_down
			self.newEvent()
		

# ------------------- Main Program Loop --------------------
if __name__ == "__main__":

		ROS_Head()
		

	
