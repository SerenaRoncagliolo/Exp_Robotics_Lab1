#!/usr/bin/env python

## @package motion
#
# It moves the robot within the map respecting the behavior

import rospy

from std_msgs.msg import String # needed for subscribing strings

## @param xmax define max dimension of the map along X
xmax = 30
## @param ymax define max dimension of the map along Y
ymax = 30
## @param xhome define X house position for the robot
xhome = 10
## @param yhome define Y house position for the robot
yhome = 10
## @param xuser define X user position for the robot
xuser = 20
## @param yuser define Y user position for the robot
yuser = 20

## callback function  callback_get_behavior
#
# subscriber callback to the behaviour topic
def callback_get_behaviour(data):
	rospy.loginfo('Executing callback behavior')
	print("Kudrett")

## main function
#
def main():
	## initialize node
	rospy.init_node('motion')
	# way for looping at the desired rate, go through the loop 100 times per second
	
	## subscriber
	rospy.Subscriber("/behaviour", String, callback_get_behaviour)	
	print("kudrett")
	rospy.spin()		

if __name__ == "__main__":
    main()
