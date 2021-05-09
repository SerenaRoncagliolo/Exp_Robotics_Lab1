#!/usr/bin/env python

## @package pointing_gesture
#
# It simulate the pointing gesture command given by the user 
# it generates pointing gestures given as an array int[] (ros message IntArray) at random times

import rospy
import time
import random

from std_msgs.msg import String # needed for publishing/subscribing strings
from std_msgs.msg import Int32 # needed for publishing/subscribing  integers
from first_assignment.msg import IntArray # needed for publishing/subscribing  [x,y] describing the position of the robot
from map2Dclass import Map2D # class to simulate map of the environment


## object Map2D, we need it to access the values describing the map 2D within which the robot is moving 
map_2D = Map2D()

## global variables
behaviour = None # used to check the current behavior of the robot  
random_time = 0.5 # used to compute random time

## publisher
# send pointing gesture, send a random position 
pub_pointing_gesture = rospy.Publisher("/pointing_gesture", IntArray, queue_size=10)


## callback function  callback_get_behavior
#
# subscriber callback to the topic /behavior
def callback_get_behaviour(data):
	global behaviour 
	behaviour = data.data

## function compute_random_position
#
# generate a random position that the robot should reach as goal when in play behavior
def compute_random_position():
	## get random position
	randX = random.randint(0,map_2D.x_max) 
	# print("x max :", map_2D.x_max)
	randY = random.randint(0,map_2D.y_max) 
	# print("y max :", map_2D.x_max)
	rand_position = [randX,randY]
	return rand_position

## main function
#
def main():
	## init node
	rospy.init_node('pointing_gesture')
	rate = rospy.Rate(100)
	
	## subscriber to topic /behavior
	# read current behavior of the robot
	rospy.Subscriber("/behavior", String, callback_get_behaviour)			

	while not rospy.is_shutdown():
		## wait random time
        	rospy.sleep(random_time*random.randint(10,50))
		## publish position
		random_position = compute_random_position()
		# print("Random position: ", random_position)
		pub_pointing_gesture.publish(random_position)
		print("NODE POINTING: goal position published: ", random_position)

		rate.sleep()
	
	
if __name__ == "__main__":
    main()
