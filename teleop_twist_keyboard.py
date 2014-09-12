#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Vector3

def getch():
    """ Return the next character typed on the keyboard """
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch


def vel_readout(lin_speed, ang_speed):
	""" Returns a string with robot velocity details """
	return 'Robot is currently traveling at linear speed: {lin}, and angular speed: {ang}.'.format(lin=lin_speed, ang=ang_speed)


controls_msg = """
Reading from keyboard and publishing to Twist!
-----------------------
Moving around:
	w 			|					forward
a 	s 	d 		|	rotate left		reverse		rotate right

Spacebar stops the robot.  Any other key does nothing.

TAB to quit.
""" 

# Binds keypresses to Neato motion -- 'keypress': linear velocity (x), angular velocity (z)
# Note: there is no y/z component of linear velocity, or x/y component of angular velocity (because Neato cannot move those ways =( )
moveBindings = {
	'w': (1, 0),	# forward
	'a': (0, 1),	# rotates left 
	's': (-1, 0),	# reverse
	'd': (0, -1),	# rotates right
	' ': (0, 0)		# stop
}

# Sets constants for scaling linear and angular velocity
lin_speed = 0.5
ang_speed = 0.5


if __name__=='__main__':
	# Sets to publish to /cmd_vel node
	pub = rospy.Publisher('cmd_vel', Twist)
	# Initiates with name teleop_node
	rospy.init_node('teleop_node', anonymous=True)

	# Set initial velocity values to 0
	x_vel = 0
	ang_vel = 0

	# while not rospy_is_shutdown():
	try:
		print controls_msg
		print vel_readout(lin_speed, ang_speed)
		while True:
			# Gets keypress in order to adjust velocities accordingly
			key = getch()
			print key
			# If key pressed is in the mapped keys dict, change velocities acordingly
			if key in moveBindings.keys():
				x_vel = moveBindings[key][0]
				ang_vel = moveBindings[key][1]
				print x_vel, ang_vel
			else:
				# If Tab is pressed, break out of everything 
				if (key=='	'):
					break
				print ('That isn\'t a valid key.  Try again')

			# Constructs and publishes the linear and angular velocities to /vel_cmd
			twist = Twist()
			twist.linear = Vector3(x_vel*lin_speed, 0, 0)
			# twist.linear.x = x_vel*lin_speed; twist.linear.y = 0; twist.linear.z = 0
			twist.angular = Vector3(0, 0, ang_vel*ang_speed)
			# twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = ang_vel*ang_speed
			print twist
			pub.publish(twist)


	# If exception happens, notify user
	except:
		print 'something messed up'

	# If uncaught exception happens, execute cleanup
	finally:
		twist = Twist()
		twist.linear = Vector3(0,0,0)
		twist.angular = Vector3(0,0,0)
		# twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
		# twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
		pub.publish(twist)










