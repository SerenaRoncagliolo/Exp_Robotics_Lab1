#!/usr/bin/env python

## @package behavior_manager
# Here is implemented the state machine that controls the switch between the behaviours of the robot
#
# A finite-state machine is a behavior model that consists of a finite number of states. 
# Based on the current state and a given input the machine performs state transitions and produces outputs
# The state machine is implemented using the smach library


import rospy
import smach
import smach_ros

## class Normal_behavior
#
# This class implement the NORMAL behaviour of the robot pet
class Normal_behavior(smach.State):
	## method init
	#
	# it initializes the state class
	def __init__(self):
	## method execute
	#
	# it executes the required actions
	def execute(self, userdata):

## class Sleep_behavior
#
# This class implement the SLEEP behaviour of the robot pet
class Sleep_behavior(smach.State):
	## method init
	#
	# it initializes the state class
	def __init__(self):
	## method execute
	#
	# it executes the required actions
	def execute(self, userdata):

## class Play_behavior
#
# This class implement the PLAY behaviour of the robot pet
class Play_behavior(smach.State):
	## method init
	#
	# it initializes the state class
	def __init__(self):
	## method execute
	#
	# it executes the required actions
	def execute(self, userdata):


