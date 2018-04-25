#!/usr/bin/env python

# ROS Node that controls vehicle navigation
# Take in user input, IMU, GPS, sonar sensor data
# outputs motor commands

import rospy
from std_msgs.msg import Float32MultiArray

def imu_callback(data):
	print(data.data)

def imu_listener():
	rospy.init_node('vehicle_nav', anonymous = True)
	
	rospy.Subscriber("sensor_data", Float32MultiArray, imu_callback)
	
	rospy.spin()
	
if __name__ == '__main__':
	imu_listener()
