#!/usr/bin/env python

# ROS Node that controls vehicle navigation
# Take in user input, IMU, GPS, sonar sensor data
# outputs motor commands

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu 

pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
vel_msg = Twist()

def imu_callback(data):
	# do stuff with the IMU data
	# print(data.orientation.x)
	vel_msg.angular.x = 10
	vel_msg.angular.y = 5
	pub.publish(vel_msg)

def vehicle_nav():
	rospy.init_node('vehicle_nav', anonymous = True)
	rospy.Subscriber("sensor_data", Imu, imu_callback)
	
	rospy.spin()

	
	
	
if __name__ == '__main__':
	try:
		vehicle_nav()
	except rospy.ROSInterruptException:
		pass
