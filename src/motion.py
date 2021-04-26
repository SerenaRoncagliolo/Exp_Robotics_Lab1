#!/usr/bin/env python

## @package motion
#
# It moves the robot within the map respecting the behavior

import rospy

# define max dimension of the map along X
xmax = 30
# define max dimension of the map along Y
ymax = 30
# define X house position for the robot
xhome = 10
# define Y house position for the robot
yhome = 10
# define X user position for the robot
xuser = 20
# define Y user position for the robot
yuser = 20


## main function
#
def main():
	## initialize node
	rospy.init_node('motion')
	rate = rospy.Rate(100)


if __name__ == "__main__":
    main()
