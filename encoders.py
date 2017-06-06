#
#	encoders_working.py - Collects encoder data from 'Encoder' Arduino and 
#	publishes it to the ROS network.
#	written partly by: T.Reese and B.Yuen
#	
# 	ROS ref: http://wiki.ros.org/common_msgs and http://wiki.ros.org/std_msgs
#	code refs: http://stackoverflow.com/questions/3305926/python-csv-string-to-array
#		

import rospy
import serial
import tf
from math import pi
from math import sqrt
from math import cos
from math import sin
from locale import atof
from time import sleep
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Imu

TH = 0.0

def imu_callback(data):
	global TH 
	e = tf.transformations.euler_from_quaternion ( [data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w] )
	TH = e[2]

#f = open('encdata0.txt', 'w')
class encoders():
	def __init__(self):
		global TH
		# Initialize 'encoders' node
		rospy.init_node('encoders')
		
		# Specify and initialize the port 'Encoder' Arduino is on 
		try:
			self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1, write_timeout=1)
		except: 
			self.ser = serial.Serial('/dev/ttyACM1', 115200, timeout=1, write_timeout=1)
		# Initialize publisher to the 'Encoders' topic  
		enc_pub = rospy.Publisher('Encoders', PointStamped, queue_size = 100)
		# Initialize publisher to the 'odom' topic  
		pub = rospy.Publisher('odom', Odometry, queue_size = 100)
		# Initialize subscriber to the 'imu_data' topic
		rospy.Subscriber('imu_data', Imu, imu_callback, queue_size = 100)

		# Variables for pulling the messages from the Arduino  
		new_pose = Odometry()
		new_position = Point()
		new_linear_velocity = Vector3()
		new_angular_velocity = Vector3()
		enc_data = PointStamped()
	
		new_pose.header.frame_id = 'odom'
		new_pose.child_frame_id = 'base_link'

		new_position.x = 0.0
		new_position.y = 0.0
		new_position.z = 0.0

		new_linear_velocity.x = 0.0
		new_linear_velocity.y = 0.0
		new_linear_velocity.z = 0.0
		
		new_angular_velocity.x = 0.0
		new_angular_velocity.y = 0.0
		new_angular_velocity.z = 0.0		

		prev_CX = 0.0
		prev_CY = 0.0
		prev_TH = 0.0

		new_pose.pose.covariance = [0.0001, 0, 0, 0, 0, 0,      
								        0, 0.0001, 0, 0, 0, 0,  
								        0, 0, 0, 0, 0, 0, 
								        0, 0, 0, 0, 0, 0,  
								        0, 0, 0, 0, 0, 0,  
								        0, 0, 0, 0, 0, 0.01]

		new_pose.twist.covariance = [0.0006, 0, 0, 0, 0, 0,      
								        0, 0.0006, 0, 0, 0, 0,  
								        0, 0, 0, 0, 0, 0, 
								        0, 0, 0, 0, 0, 0,  
								        0, 0, 0, 0, 0, 0,  
								        0, 0, 0, 0, 0, 0.01]

		previous_time =  rospy.Time()

		prevE1 = 0.0
		prevE2 = 0.0
		prevE3 = 0.0


		while not rospy.is_shutdown():	
			if self.ser.inWaiting() > 0:						# If message comes
				# Get ROS time
				new_pose.header.stamp = rospy.get_rostime() - rospy.Duration(0.0002) 
				enc_data.header.stamp = new_pose.header.stamp
				ard_msg = self.ser.readline()					# Read line of incoming data
				ard_msg = ard_msg.rstrip()						# Strip newline character from end
				temp = ard_msg.split(',')						# Separate comma delimited values
				num_vals = len(temp)
				if num_vals == 4:
					#Calculate distance of wheels
					try:
						E1 = atof(temp[0])
						E2 = atof(temp[1])
						E3 = atof(temp[2])
						checksum = atof(temp[3])
					except: 
						#print "fail"
						continue
					enc_data.point.x = E1	
					enc_data.point.y = E2
					enc_data.point.z = E3
					enc_pub.publish(enc_data)
					#print [E1,E2,E3,checksum] 
					if ((abs(E1 - prevE1) < 100.0) and (abs(E2 - prevE2) < 100.0) and (abs(E3 - prevE3) < 100.0)):
						if checksum == (abs(E1 + E2 + E3) % 1000):
					
							W1 = ((2.0*pi*0.05)/768.0)*(E1)
							W2 = ((2.0*pi*0.05)/768.0)*(E2)
							W3 = ((2.0*pi*0.05)/768.0)*(E3)
						
		
							#Calculate position vector
							#CX = ((1.0/3.0)*(W3+W2)) - ((2.0/3.0)*(W1))
							#CY = (sqrt(3.0)/3.0)*(W3 - W2)

							CX = (sqrt(3.0)/3.0)*(W3 - W2)
							CY = ((-1.0/3.0)*(W3+W2)) + ((2.0/3.0)*(W1)) 
				
							DX = CX - prev_CX
							DY = CY - prev_CY
				
							new_pose.pose.covariance[0] += 0.03*abs(DX)
							new_pose.pose.covariance[7] += 0.03*abs(DY)

							#Set position
							new_position.x = new_position.x + DX*cos(TH) - DY*sin(TH)
							new_position.y = new_position.y + DX*sin(TH) + DY*cos(TH)
				
				
							#Calculate linear velocity
							DT = (new_pose.header.stamp.secs - previous_time.secs) + ((new_pose.header.stamp.nsecs -  previous_time.nsecs)/1.0e9)
							new_linear_velocity.x = DX/DT
							new_linear_velocity.y = DY/DT
				
							#Calculate angular velocity
							new_angular_velocity.z = (TH - prev_TH)/DT
							#QUAT = tf.transformations.unit_vector (tf.transformations.quaternion_from_euler(0, 0, TH) )
							#QUAT = tf.transformations.quaternion_from_euler(0, 0, TH+(-pi/2))
							QUAT = tf.transformations.quaternion_from_euler(0, 0, TH)

							new_pose.pose.covariance[35] += 0.4*abs(TH - prev_TH)

							#f.write(str(new_position.x)+','+str(new_position.y)+','+str(new_linear_velocity.x)+','+str(new_linear_velocity.y)+'\n')

							new_pose.pose.pose.position = new_position		# Copy new_encvals into the pose field of message
							new_pose.pose.pose.orientation = Quaternion(*QUAT)
							new_pose.twist.twist.linear = new_linear_velocity
							new_pose.twist.twist.angular = new_angular_velocity
							pub.publish(new_pose)							# Publish message
							prev_CX = CX
							prev_CY = CY
							previous_time.secs = new_pose.header.stamp.secs
							previous_time.nsecs = new_pose.header.stamp.nsecs
							prev_TH = TH

							prevE1 = E1
							prevE2 = E2 
							prevE3 = E3
		#f.close()
		
if __name__ == "__main__" :
	encoders()


