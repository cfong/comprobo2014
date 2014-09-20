#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan

# Whee globals!
pub = None

def range_is_valid(scan_msg):
	"""Returns True if laser scan message range is within valid range"""
	if scan_msg >= 0.1 and scan_msg < 7.0:
		return True

	return False

def scan_recieved(msg):
	"""Call back function for LaserScan messages"""
	global pub 

	# Default behavior
	pub.publish(Twist(linear=Vector3(x=0.5)))

	front_scan = [msg.ranges[x] for x in range(0,10) if range_is_valid(msg.ranges[x])]

	# Scans front left and right 'corners' to see if there is stuff in the way
	right_scan = [msg.ranges[x] for x in range(315-20, 315+20) if range_is_valid(msg.ranges[x])]
	left_scan = [msg.ranges[x] for x in range(135-20, 135+20) if range_is_valid(msg.ranges[x])]

	right_avg = sum(right_scan)/len(right_scan)
	left_avg = sum(left_scan)/len(left_scan)

	print 'RIGHT ', right_avg
	print 'LEFT ', left_avg

	if sum(front_scan)/len(front_scan) < 1:
		print ('Something here!')
		pub.publish(Twist(linear=Vector3(x=0)))

		if right_avg < left_avg:
			pub.publish(Twist(angular=Vector3(z=0.2)))
		else:
			pub.publish(Twist(angular=Vector2(z=-0.2)))


def test():
	"""Main run loop for testing."""
	global pub
	# Initiates as 'wall_follow' node
	rospy.init_node('obstacle_avoid', anonymous=True)

	# Sets to publish to 'cmd_vel' topic and subscribe to laserscan rangefinder ('scan') topic
	pub = rospy.Publisher('/cmd_vel', Twist)
	sub = rospy.Subscriber('/scan', LaserScan, scan_recieved)

	# Sets publish rate to 10 hz 
	r = rospy.Rate(10)

	while not rospy.is_shutdown():
		r.sleep()


if __name__ == '__main__':
	try:
		test()

	except rospy.ROSInterruptException: pass
