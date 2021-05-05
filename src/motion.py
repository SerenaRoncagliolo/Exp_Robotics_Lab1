#!/usr/bin/env python

## @package motion
#
# It moves the robot within the map respecting the behavior

import rospy
import time
import random

from std_msgs.msg import String # needed for subscribing strings
from std_msgs.msg import Int32 # needed for publishing integers
from first_assignment.msg import IntArray # I need to publish/subscribe [x,y]
from classes.map2Dclass import Map2D # class to simulate map of the environment

## all these param are now saved in the class Map2D in map2Dclass.py
## xmax define max dimension of the map along X
#xmax = 30
## ymax define max dimension of the map along Y
#ymax = 30
## xhome define X house position for the robot
#xhome = 10
## yhome define Y house position for the robot
#yhome = 10
## xuser define X user position for the robot
#xuser = 20
## yuser define Y user position for the robot
#yuser = 20

## global variables
behaviour = None
at_home = False
goal_position = None

random_time = 0.5 # NB remember to get param from launch file

## object for access the values of the map2D
map_2D = Map2D()

## publisher of actual position of the robot
#
# publish actual position on the topic actual_position_robot which is subscibed by behavior_manager in Sleep state
pub_actual = rospy.Publisher("/actual_position_robot",IntArray,queue_size=10)

## callback function  callback_get_behavior
#
# subscriber callback to the behaviour topic
def callback_get_behaviour(data):
	#rospy.loginfo('NODE MOTION: Executing callback behavior')
	global behaviour 
	behaviour = data.data

## function update_position
#
# update actual position of the robot with the given one
# this function is now implemented in the class Map2D
def update_position(x,y):
	global map_2D
	map_2D.x_actual=x
	map_2D.y_actual=y

## function move_random
#
# the robot moves randomly when in the NORMAL state
def move_normal():
	rospy.loginfo("NODE MOTION: function to move in normal mode to a random position")
	global map_2D
	## get random position
	randX = random.randint(0,map_2D.x_max) 
	randY = random.randint(0,map_2D.y_max) 
	randPos = [randX,randY]
	## update actual position
	update_position(randPos[0],randPos[1])
	## wait random time to simulate the robot has moved and reached position
	rospy.sleep(random_time*random.randint(3,18))

## function move_reach_user
#
# the robot reached the user when in the PLAY state
def move_reach_user():
	## get random position	
	global map_2D
	update_position(map_2D.x_home,map_2D.y_home)


## function move_sleep_position
#
# movement in the SLEEP state
def move_sleep_position():
	
	global at_home
	global map_2D
	## go to the home position
	if not at_home:
		rospy.loginfo("NODE MOTION: move into sleep position")
	        ## wait random time to simulate reaching the point
	        rospy.sleep(random_time*random.randint(6,30))
	        update_position(map_2D.x_home,map_2D.y_home)
		rospy.loginfo('NODE MOTION:  The robot asleep at home position')
	        at_home = True

## function callback_get_position
#
# subscriber callback position
def callback_get_position(position):
    global goal_position
    goal_position = position.data


## main function
#
def main():
	## initialize node
	rospy.init_node('motion')
	
	# we impose that the initial position corresponds to home position 
	map_2D.x_actual=map_2D.x_home 
	map_2D.y_actual=map_2D.y_home 
	rospy.loginfo('NODE MOTION: Initial x position: %d', map_2D.x_actual)
	rospy.loginfo('NODE MOTION: Initial y position: %d', map_2D.y_actual)

	## pub initial position
    	pub_actual.publish([map_2D.x_actual,map_2D.y_actual])
	
	global at_home
	global goal_position
	global map_2D
	#global xuser
	#global yuser
	
	## subscriber
	rospy.loginfo('Subscriber /behavior')
	rospy.Subscriber("/behavior", String, callback_get_behaviour)		
	rospy.Subscriber("/pointing_gesture",IntArray, callback_get_position)

	rate = rospy.Rate(100)

	## move according to the behaviour
	while not rospy.is_shutdown():
		# check robot behavior
		if(behaviour == "sleep"):
			# rospy.loginfo('NODE MOTION: enter sleep behavior')
			## the robot moves to predefined location
			move_sleep_position()
			## ignore pointing command
            		if not goal_position == None:
                		goal_position = None
			
		else:
			if(behaviour == "normal"):
				rospy.loginfo('NODE MOTION: enter normal state')
				at_home = False
				## robot is moving randomly, call function move_normal()
				move_normal()
				## ignore pointing command
            			if not goal_position == None:
                			goal_position = None
				rospy.loginfo('NODE MOTION: robot reached random position')
		
			else:
				if(behaviour == "play"):
					rospy.loginfo("NODE MOTION: behaviour play")
					## the robot goes to the user location
					## we first check it is not there already
					if not ((map_2D.x_actual,map_2D.y_actual) == (map_2D.xuser,map_2D.yuser)):
						## call function move_reach_user() to reach the user posotion
						move_reach_user() 
						## wait random time to simulate the robot has moved and reached position
						rospy.sleep(random_time*random.randint(3,10))
						rospy.loginfo('NODE MOTION: play - user position reached')
					else:
						## waits for a pointing gesture
						## goes in the pointed location
						print('NODE MOTION: pointing gesture still to implement')
				
	
	rospy.spin()		

if __name__ == "__main__":
    main()
