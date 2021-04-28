#!/usr/bin/env python

## @package behavior_manager
# Here is implemented the state machine that controls the switch between the behaviours of the robot
#
# A finite-state machine (FSM) is a behavior model that consists of a finite number of states. 
# Based on the current state and a given input the machine performs state transitions and produces outputs
# The state machine is implemented using the smach library


import rospy
import smach
import smach_ros
import time
import random

from std_msgs.msg import String # need for publishing strings

# publisher pub_behavior
#
# it declares that the node publishes to the behavior topic using the message type String.
# the queue_size argument limits the amount of queued messages if any subscriber is not receiving them fast enough.
pub_behavior = rospy.Publisher('/behavior', String, queue_size=10)




## class Normal_behavior
#
# This class implement the NORMAL behaviour of the robot pet
# The robot moves randomly within the map 
# - If it receives a "play" command the FSM should go into PLAY state 
# - If the sleep timer is triggered the FSM should go into SLEEP state
class Normal_behavior(smach.State):
	## method init
	#
	# it initializes the state class
	def __init__(self):
		smach.State.__init__(self, 
		                     outcomes=['start_sleep','start_play']
		                    )	
		self.counter = 0	
	## method execute
	#
	# it executes the required actions
	def execute(self, userdata):
		rospy.sleep(2)
		self.counter = random.randint(1,2) 
		print('kudrettttttttttt')
		rospy.loginfo('Executing state NORMAL')
		return 'start_play'
		
## class Sleep_behavior
#
# This class implement the SLEEP behaviour of the robot pet
# The robot sleeps (SLEEP state)for a random period of time, then it moves to NORMAL state
## class Sleep_behavior
#
# This class implement the SLEEP behaviour of the robot pet
# The robot sleeps (SLEEP state)for a random period of time, then it moves to NORMAL state
class Sleep_behavior(smach.State):
	## method init
	#
	# it initializes the state class
	def __init__(self):
		smach.State.__init__(self, 
                             outcomes=['stop_sleep']
                            )
	## method execute
	#
	# it executes the required actions
	def execute(self, userdata):
		rospy.sleep(2)
		return 'stop_sleep'



## class Play_behavior
#
# This class implement the PLAY behavior of the robot pet
# It moves the robot to the predefined (X, Y) location within the map and moves it back to the user.
class Play_behavior(smach.State):
	## method init
	#
	# it initializes the state class
	def __init__(self):
		smach.State.__init__(self, 
		                     outcomes=['stop_play']
		                    )
		self.rate = rospy.Rate(200)  
	## method execute
	#
	# it executes the required actions
	def execute(self, userdata):
		rospy.sleep(0.5)
		pub_behavior.publish("play") 
		print('publisher kudretttttt')
		#rospy.loginfo('Executing state PLAY')
		#rospy.sleep(timescale*random.randint(60,120))
		return 'stop_play'

def main():
	rospy.init_node("behavior_manager")

	## Create state machine	
	sm = smach.StateMachine(outcomes=['container_interface'])
	## machine container
	with sm:
	## add states to the container,
		smach.StateMachine.add('NORMAL', Normal_behavior(), transitions={'start_sleep':'SLEEP','start_play':'PLAY'})
		smach.StateMachine.add('SLEEP', Sleep_behavior(), transitions={'stop_sleep':'NORMAL'})	
 		smach.StateMachine.add('PLAY', Play_behavior(), transitions={'stop_play':'NORMAL'})	
	
	## Create and start the introspection server for visualization
   	sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    	sis.start()

	# Execute SMACH plan
	outcome = sm.execute()
		
	## Wait for ctrl-c to stop the application
    	rospy.spin()
    	sis.stop()


if __name__ == "__main__":
    main()

