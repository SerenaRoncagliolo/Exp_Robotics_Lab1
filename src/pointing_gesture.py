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
from classes.map2Dclass import Map2D # class to simulate map of the environment

## xmax and ymax are now read from Map2D
## xmax define max dimension of the map along X
#xmax = 30
## ymax define max dimension of the map along Y
#ymax = 30

## create object Map2D
map_2D = Map2D()

## global variables
behaviour = None
random_time = 0.5 # NB remember to get param from launch file

## publisher
# send pointing gesture, send a random position 
pub_pointing_gesture = rospy.Publisher("/pointing_gesture", IntArray, queue_size=10)


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
	randX = random.randint(0,map_2D.x_max) 
	randY = random.randint(0,map_2D.y_max) 
	rand_position = [randX,randY]
	return rand_position

## main function
#
def main():
	## init node
	rospy.init_node('pointing_gesture')
	rate = rospy.Rate(100)
	
	## subscriber
	# read behavior
	rospy.loginfo('Subscriber /behavior')
	rospy.Subscriber("/behavior", String, callback_get_behaviour)			
	
	while not rospy.is_shutdown():
		 ## wait random time
        	rospy.sleep(random_time*random.randint(30,100))
		## publish position
		pub_pointing_gesture.publish(compute_random_position())
		rate.sleep()
	

if __name__ == "__main__":
    main()
