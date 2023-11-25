std_msgs/Header header                      # header
geometry_msgs/Pose2D position_point         # coordinates of the feature point
uint8 source                                # source of the lane
uint8 position_vehicle                      # position of lane with respect to vehicle


# source
uint8 ST_MDI_RANGE_RADAR = 0
uint8 ST_RADAR = 1
uint8 ST_LIDAR = 2
uint8 ST_LONG_RANGE_RADAR = 3
uint8 ST_V2X = 4
uint8 ST_CAMERA = 5
uint8 ST_FUSIONv = 6
uint8 ST_UNKNOWN = 99

# position
uint8 POSITION_UNDEFINED = 0
uint8 POSITION_ADJACENT_LEFT_RIGHT = 1
uint8 POSITION_ADJACENT_LEFT = 2
uint8 POSITION_EGO_LEFT = 3
uint8 POSITION_EGO_RIGHT = 4
uint8 POSITION_ADJACENT_RIGHT = 5
uint8 POSITION_ADJACENT_RIGHT_LEFT = 6