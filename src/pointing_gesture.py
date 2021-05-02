#!/usr/bin/env python

## @package pointing_gesture
#
# It simulate the pointing gesture command given by the user 
# it generates pointing gestures (as IntArray) at random times

import rospy
import time
import random

from std_msgs.msg import String # needed for subscribing strings
from std_msgs.msg import Int32 # needed for publishing integers
from first_assignment.msg import IntArray # I need to publish/subscribe [x,y]


## global variables
behaviour = None


## callback function  callback_get_behavior
#
# subscriber callback to the behaviour topic
def callback_get_behaviour(data):
	rospy.loginfo('Executing callback behavior')
	global behaviour 
	behaviour = data.data
	print("Current behaviour: ", behaviour)


## function compute_random_position
#
# generate a random position to reach
def compute_random_position():
	## get random position
	randX = random.randint(0,xmax) 
	randY = random.randint(0,ymax) 
	rand_position = [randX,randY]
	return rand_position



## main function
#
def main():
	## init node
	rospy.init_node('pointing_gesture')
	rate = rospy.Rate(100)

	## subscriber
	rospy.loginfo('Subscriber /behavior')
	rospy.Subscriber("/behavior", String, callback_get_behaviour)			

	rospy.spin()		

if __name__ == "__main__":
    main()
