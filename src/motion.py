#!/usr/bin/env python

## @package motion
#
# It moves the robot within the map respecting the behavior

import rospy
import time
import random

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

## global variables
behaviour = None
at_home = False

timescale = 0.5 # next get param from launch file



## callback function  callback_get_behavior
#
# subscriber callback to the behaviour topic
def callback_get_behaviour(data):
	rospy.loginfo('Executing callback behavior')
	global behaviour 
	behaviour = data.data
	print("Current behaviour: ", behaviour)

## function update_position
#
# update actual position of the robot with the given one
def update_position(x,y):
	x_actual=x
	y_actual=y

## function move_random
#
# the robot moves randomly when in the NORMAL state
def move_normal():
	## get random position
	randX = random.randint(0,xmax) 
	randY = random.randint(0,ymax) 
	randPos = [randX,randY]
	## update actual position
	update_position(randPos[0],randPos[1])

## function move_reach_user
#
# the robot reached the user when in the PLAY state
def move_reach_user():
	## get random position	
	update_position(xhome,yhome)


## function move_sleep_position
#
# movement in the SLEEP state
def move_sleep_position():
    global at_home
    ## go to the home position
    if not at_home:
        ## wait random time to simulate reaching the point
        rospy.sleep(timescale*random.randint(6,18))
        update_position(xhome,yhome)
        at_home = True


## main function
#
def main():
	## initialize node
	rospy.init_node('motion')
	
	# we impose that the initial position corresponds to home position 
	x_actual=xhome 
	y_actual=yhome
	rospy.loginfo('Initial x position: %d', x_actual)
	rospy.loginfo('Initial y position: %d', y_actual)

	## subscriber
	rospy.loginfo('Subscriber /behavior')
	rospy.Subscriber("/behavior", String, callback_get_behaviour)		
	
	## move according to the behaviour
	while not rospy.is_shutdown():
		# check robot behavior
		if(behaviour == "normal"):
			## robot is moving randomly, call function move_normal()
			move_normal()
			rospy.loginfo('robot reached random position')
			## wait random time to simulate the robot has moved and reached position
			rospy.sleep(timescale*random.randint(3,18))
			rospy.loginfo('behavior NORMAL')
		
		else:
			if(behaviour == "play"):
				rospy.loginfo("enter if behaviour play")
				## the robot goes to the user location
				## we first check it is not there already
				if not ((x_actual,y_actual) == (xuser,yuser)):
					## call function move_reach_user() to reach the user posotion
					move_reach_user() 
					rospy.loginfo('user position reached')
					## wait random time to simulate the robot has moved and reached position
					rospy.sleep(timescale*random.randint(3,18))
					rospy.loginfo('behavior PLAY')
				else:
					## waits for a pointing gesture
					## goes in the pointed location
					print('final kudret')
			else:
				if(behaviour == "sleep"):
					rospy.loginfo("kudrettttt sleep")
					# the robot moves to predefined location
					## wait random time to simulate reaching the point
					move_sleep_position()
					rospy.loginfo('The robot can now sleep')

	
	rospy.spin()		

if __name__ == "__main__":
    main()
