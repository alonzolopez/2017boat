#!/usr/bin/env python

# User Input: Compass heading (degrees)
# Sensor Input: Magnetometer and integrated speed data from IMU
# Output: Output motor commands to keep the boat running
# in the desired direction

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu 
import math

pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
vel_msg = Twist()
speed = 0 # speed in m/s
t = 0 # time in milliseconds
old_t = 0 # old time in milliseconds

desired_w = 15 # propeller rotational velocity in rev/s
desired_heading = 0.0 # due north

def imu_callback(data):
	# Every time we get IMU data, 
	# we want to check our orientation and
	# calculate our speed (by integrating 
	# acceleration provided by the IMU)
	actual_heading = remap_heading(data.orientation.x) # compass heading provided by IMU
	error_heading = desired_heading - actual_heading
	kp = 0.1
	bias = kp*error_heading
	vel_msg.angular.x = desired_w + bias
	vel_msg.angular.y = desired_w - bias
	pub.publish(vel_msg)
	
def remap_heading(heading):
	# remaps the data.orientation.x readings so that they
	# are centered around 0 and
	# turning left goes towards - 90 degrees 
	# and turning right goes towards + 90 degrees
	# otherwise, just left of 0 is 359 degrees
	if heading > 180:
		return heading-360
	else:
		return heading
	
def update_speed(x_accel, y_accel, dt):
	dv_x = x_accel*dt
	dv_y = y_accel*dt
	speed = speed + math.sqrt(dv_x**2+dv_y**2)
	old_t = t # update old time
	
	
	
	
def vehicle_nav():
	rospy.init_node('vehicle_nav', anonymous = True)
	rospy.Subscriber("sensor_data", Imu, imu_callback)
	rospy.spin()
	
if __name__ == '__main__':
	try:
		vehicle_nav()
	except rospy.ROSInterruptException:
		pass
