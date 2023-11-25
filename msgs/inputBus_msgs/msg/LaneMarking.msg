uint16 id							# line id
float32 confidence					# confidence that line marking exists
										
uint8 type							# type of line marking
uint8 position						# position of line marking
										
uint16 numPoints					# number of points corresponding to line
										
geometry_msgs/Point[] imagePoints	# points corresponding to line, image space
										
geometry_msgs/Point[] worldPoints
uint32 uiIndexOfEgo					# index in sequence of crossing y axis (position of origin)
uint8 source						# source of line
uint8 posWRTego    				# position with respect to source

# source
uint8 ST_MDI_RANGE_RADAR = 0
uint8 ST_RADAR = 1
uint8 ST_LIDAR = 2
uint8 ST_LONG_RANGE_RADAR = 3
uint8 ST_V2X = 4
uint8 ST_CAMERA = 5
uint8 ST_FUSIONv = 6
uint8 ST_HP_MAP = 7
uint8 ST_UNKNOWN = 99



# type
uint8 TYPE_UNDEFINED = 0
uint8 TYPE_WHITE_SOLID = 1
uint8 TYPE_WHITE_DASHED = 2
uint8 TYPE_UNKNOWN = 3
uint8 TYPE_TRAJECTORY = 4
uint8 TYPE_YELLOW_SOLID = 5
uint8 TYPE_YELLOW_DASHED = 6
uint8 TYPE_ROAD_BOUNDARY = 7


# position
uint8 POSITION_UNDEFINED = 0
uint8 POSITION_ADJACENT_LEFT_RIGHT = 1
uint8 POSITION_ADJACENT_LEFT = 2
uint8 POSITION_EGO_LEFT = 3
uint8 POSITION_EGO_RIGHT = 4
uint8 POSITION_ADJACENT_RIGHT = 5
uint8 POSITION_ADJACENT_RIGHT_LEFT = 6


# posWRTego
uint8 POS_UNKNOWN = 0
uint8 POS_AHEAD = 1
uint8 POS_BEHIND = 2
