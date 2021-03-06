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

desired_w = 30 # propeller rotational velocity in rev/s
desired_heading = 0.0 # due north

# filter variables
x_1 = 0
x_2 = 0
x_3 = 0
x_4 = 0
x_5 = 0
y_1 = 0 
y_2 = 0
y_3 = 0
n = 9
y_0 = 0

def iirfilter(x_0):
	# have to let this run for about a second before the averaging works out
	global x_1, x_2, x_3, x_4, x_5, y_1, y_2, y_3, y_0, n
	b_0 = 1.0/n
	b_1 = 1.0/n
	b_2 = 1.0/n
	b_3 = 1.0/n
	b_4 = 1.0/n
	b_5 = 1.0/n
	c_1 = 1.0/n
	c_2 = 1.0/n
	c_3 = 1.0/n
	
	y_0 = b_0*x_0 + b_1*x_1 + b_2*x_2 + b_3*x_3 + b_4*x_4 + b_5*x_5 + c_1*y_1 + c_2*y_2 + c_3*y_3 
	
	y_3 = y_2
	y_2 = y_1
	y_1 = y_0
	x_5 = x_4
	x_4 = x_3
	x_3 = x_2
	x_2 = x_1
	x_1 = x_0
	
	return y_0

def imu_callback(data):
	# Every time we get IMU data, 
	# we want to check our orientation and
	# calculate our speed (by integrating 
	# acceleration provided by the IMU)
	actual_heading = remap_heading(data.orientation.x) # compass heading provided by IMU
	filtered_heading = iirfilter(actual_heading)
	error_heading = desired_heading - filtered_heading
	kp_thrust = 0.1
	bias = kp_thrust*error_heading
	kp_servo = 0.5
	vel_msg.angular.x = desired_w + bias #motor rpm cmd
	vel_msg.angular.y = desired_w - bias #motor rpm cmd
	vel_msg.angular.z = kp_servo*error_heading #servo angle command
	vel_msg.linear.x = filtered_heading # for filter viewing purposes
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
