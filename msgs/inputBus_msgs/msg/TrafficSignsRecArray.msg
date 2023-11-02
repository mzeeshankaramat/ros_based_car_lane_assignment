std_msgs/Header header
time creation_timestamp
uint16 number_of_signs
TrafficSignsRec[] trafficSigns
uint8 source_device_id
uint8 source_type

# source_type
uint8 ST_MDI_RANGE_RADAR = 0
uint8 ST_RADAR = 1
uint8 ST_LIDAR = 2
uint8 ST_LONG_RANGE_RADAR = 3
uint8 ST_V2X = 4
uint8 ST_CAMERA = 5
uint8 ST_FUSION = 6
uint8 ST_UNKNOWN = 99

