#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan

# Whee globals!
pub = None

# Sets default target distance to wall (in meters)
target_dist = 1.0	

def wall_nearby():
	"""Determine if there is a wall within nearby range. Return 'LEFT', 'RIGHT', 'FRONT', 'BACK' (with respect to robot)"""
	wall_range = 2;

def follow(wall_dir, msg):
	"""Implement wall-following behavior (parallel to wall) for specified wall direction"""
	global pub

	base_turn_speed = 1
	tol = 1.5e-2

	# The scanning range that corresponds to each side (assuming +/-45 degrees)
	scan_ranges = {
		'LEFT': (range(45, 50), range(130, 135)),
		'RIGHT': (range(225, 230), range(310, 315)),
		'FRONT': (range(315, 320), range(40, 45)),
		'BACK': (range(135, 140), range(220, 225))
	}

	# Looks at scans from side closest to specified wall direction and calculates difference in scans +/- 45 degrees from presumed center
	side_front = [msg.ranges[x] for x in scan_ranges[wall_dir][0] if range_is_valid(msg.ranges[x])]
	side_back = [msg.ranges[x] for x in scan_ranges[wall_dir][1] if range_is_valid(msg.ranges[x])]
	side_dif = (sum(side_front)/len(side_front)) - (sum(side_back)/len(side_back))
	print('*********', side_dif, '********')
	print('*********', side_front, '********')
	print('*********', side_back, '********')

	If difference in sides is below tolerance, go forward.  Elsewise, turn in logical direction
	if abs(side_dif) <= tol:
		print ('Close enough -- theoretically moving forward')
		pub.publish(Twist(linear=Vector3(x=.2)))
	else:
		if side_dif < 0:
			print('Theoretically turning right')
		else:
			print('Theoretically turning left')
		pub.publish(Twist(angular=Vector3(z=side_dif*base_turn_speed)))

	print 'Side difference: ', side_dif
	print '*********************************************************'

def range_is_valid(scan_msg):
	"""Returns True if laser scan message range is within valid range"""
	if scan_msg > 0.05 and scan_msg < 7.0:
		return True

	return False

def scan_recieved(msg):
	"""Callback function for msg of type sensor_msgs/LaserScan"""
	global pub 

	base_turn_speed = 4
	tol = 1.5e-2

	if len(msg.ranges) != 360:
		print 'Unexpected laser scan message'
		return

	# follow(wall_nearby(), msg)
	follow('LEFT', msg)

def test():
	"""Main run loop for testing."""
	global pub
	# Sets to publish to 'cmd_vel' topic and subscribe to laserscan rangefinder ('scan') topic
	pub = rospy.Publisher('/cmd_vel', Twist)
	sub = rospy.Subscriber('/scan', LaserScan, scan_recieved)
	# Initiates as 'wall_follow' node
	rospy.init_node('wall_follow', anonymous=True)
	# Sets publish rate to 10 hz 
	r = rospy.Rate(10)

	while not rospy.is_shutdown():
		r.sleep()


if __name__ == '__main__':
	try:
		test()

	except rospy.ROSInterruptException: pass
