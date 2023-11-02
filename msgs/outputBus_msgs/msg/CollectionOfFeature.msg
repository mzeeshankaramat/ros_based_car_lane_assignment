Feature[] collectionOfFeatures                  # Lane represented as Collection of features 

uint8 type					# type of line marking
uint16 id					# line id
float32 confidence				# confidence that line marking exists
uint32 uiIndexOfEgo				# index in sequence of crossing y axis (position of origin)
uint8 position_vehicle                      	# position of lane with respect to vehicle
string frame_id
uint8 source					# source of lane

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

# source
uint8 ST_MDI_RANGE_RADAR = 0
uint8 ST_RADAR = 1
uint8 ST_LIDAR = 2
uint8 ST_LONG_RANGE_RADAR = 3
uint8 ST_V2X = 4
uint8 ST_CAMERA = 5
uint8 ST_FUSIONv = 6
uint8 ST_UNKNOWN = 99
