uint8 sensorSource
inputBus_msgs/TrafficLamp[] trafficLamps
uint16 number_of_lights
time timestamp                          # timestamp of when this phase was detected


# sensorSource
uint8 ST_MDI_RANGE_RADAR = 0
uint8 ST_RADAR = 1
uint8 ST_LIDAR = 2
uint8 ST_LONG_RANGE_RADAR = 3
uint8 ST_V2X = 4
uint8 ST_CAMERA = 5
uint8 ST_FUSIONv = 6
uint8 ST_UNKNOWN = 99