std_msgs/Header header
time timestamp              # in mikroseconds

uint8 numFreespaces         
SFreeSpaceExt[] freespaces  
uint8 source_device_id                    # id of source device
uint8 source_type                         # sensor type  

# source_type
uint8 ST_MDI_RANGE_RADAR = 0
uint8 ST_RADAR = 1
uint8 ST_LIDAR = 2
uint8 ST_LONG_RANGE_RADAR = 3
uint8 ST_V2X = 4
uint8 ST_CAMERA = 5
uint8 ST_FUSION = 6
uint8 ST_UNKNOWN = 99