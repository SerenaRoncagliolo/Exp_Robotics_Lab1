import rospy

## class Map2D
#
# we use it to define the map, because it is used by different nodes 
class PetMap():
    ## The constructor
    #  @param self The object pointer
    def __init__(self):
        # initialization of dimension and positions on the Map
        self.x_max = 30
        self.y_max = 30
        self.x_home = 10
        self.y_home = 10
        self.x_actual = self.xhome
        self.y_actual = self.thome

    ## method updateMap
    # @param x X position
    # @param y Y position
    def update_position(self,x,y):
        self.x_actual = x
        self.y_actual = y
