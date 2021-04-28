#!/usr/bin/env python

## @package voice_command
#
# It simulate the voice "play" commands given by the user 
# While it's moving in NORMAL behavior, the robot should listen to the user voice commands and enter PLAY behavior when requested

import rospy
import random

from std_msgs.msg import String


## define parameters
random_time = 0.5 # NB remember to get param from launch file
behaviour = None

## publisher pub_voice_command
#
# the node publishes on the voice_command topic using a message of type String.
pub_voice_command = rospy.Publisher('/voice_command', String, queue_size=10)

## callback function  callback_get_behavior
#
# subscriber callback to the behaviour topic
def callback_get_behaviour(data):
	rospy.loginfo('Executing callback behavior')
	global behaviour 
	behaviour = data.data

def main():
	## init node
	rospy.init_node('voice_command')

	print("xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx")
	
	## the robot enters play mode only from normal state, check behavior using subscriber to topic behavior
	## subscriber
	rospy.loginfo('Subscriber /behavior')
	rospy.Subscriber("/behavior", String, callback_get_behaviour)	
	rate = rospy.Rate(100)


	while not rospy.is_shutdown():
		if(behaviour == "normal"):
			## the command is given at random time
			rospy.sleep(random_time*random.randint(60,240)) # here we need higher values 
			## publish the voice command for playing
			pub_voice_command.publish("play") 
			
		#rate.sleep()

	rospy.spin()

if __name__ == "__main__":
	main()

