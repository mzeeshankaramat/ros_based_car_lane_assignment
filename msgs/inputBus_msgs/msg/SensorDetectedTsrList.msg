std_msgs/Header header                  # ROS Header information
                                            # uint32 seq, time stamp, string frame_id
time creation_timestamp                 # time of creation of the list (start_scan_time)
                                            # secs/nsecs unsigned 32-bit ints

uint8 source_device_id                  # id of source device
uint8 source_type                       # sensor type

uint16 number_of_objects                # number of objects
TSR[] object_list                       # list of objects



# source_type
uint8 ST_MDI_RANGE_RADAR = 0
uint8 ST_RADAR = 1
uint8 ST_LIDAR = 2
uint8 ST_LONG_RANGE_RADAR = 3
uint8 ST_V2X = 4
uint8 ST_CAMERA = 5
uint8 ST_FUSION = 6
uint8 ST_HDMAP = 7
uint8 ST_UNKNOWN = 99