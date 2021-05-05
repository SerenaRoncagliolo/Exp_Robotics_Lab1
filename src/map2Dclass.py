import rospy

## class Map2D
#
# we use it to define the map, because it is used by different nodes 
class Map2D():
	## class constructor
	def __init__(self):
		# initialization of dimension and positions on the Map
		self.x_max = 30 # define max dimension of the map along X
		self.y_max = 30 # define max dimension of the map along Y
		self.x_home = 10 # define X house position for the robot
		self.y_home = 10 # define Y house position for the robot
		self.x_user = 20 # define X user position for the robot
		self.y_user = 20 # define Y user position for the robot
		self.x_actual = self.x_home 
		self.y_actual = self.y_home

	## method update_position
	# @param x X position
	# @param y Y position
	#def update_position(self,x,y):
	#	self.x_actual = x 
	#	self.y_actual = y
