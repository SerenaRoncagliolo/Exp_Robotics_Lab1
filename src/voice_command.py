#!/usr/bin/env python

## @package voice_command
#
# It simulate the voice commands given by the user 

import rospy
import time
import random

from std_msgs.msg import String

## global variables
behaviour = None
random_time = 0.5 # NB remember to get param from launch file

## publisher pub_command_voice
#
# the node publishes on the behavior topic using a message of type String.
# the queue_size argument limits the amount of queued messages if any subscriber is not receiving them fast enough.
pub_command_voice = rospy.Publisher("/voice_command", String, queue_size=10)

## callback function  callback_get_behavior
#
# subscriber callback to the behaviour topic
def callback_get_behaviour(data):
	rospy.loginfo('Executing callback behavior')
	global behaviour 
	behaviour = data.data
	print("Current behaviour: ", behaviour)


## main function
#
def main():
	## initialize node
	rospy.init_node('voice_command')
	
	## we can enter PLAY behavior only from normal mode
	## check if the robot is in normal mode using subscriber to topic behavior
	## subscriber
	rospy.loginfo('Subscriber /behavior')
	rospy.Subscriber("/behavior", String, callback_get_behaviour)
	while not rospy.is_shutdown():
		## wait random time
        	rospy.sleep(random_time*random.randint(1,24))
		# check robot behavior
		if(behaviour == "normal"):
			## publisher, give command to start playing
			rospy.loginfo("pub on behavior")
			pub_command_voice.publish("play")


if __name__ == "__main__":
	main()

