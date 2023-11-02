# inputBus_msgs/TrafficLightV2X[] trafficLightV2x              # V2X-information for the traffic light, if exists
uint8 number_of_V2X                                     # zero or one element
inputBus_msgs/TrafficLightSensor[] trafficLightSensor        # camera based information about the traffic light, if exists
uint8 number_of_sensor                                  # zero or one element
inputBus_msgs/TrafficLightGeometry geometry                  # can be used to define the position of traffic lights and stop lines in ego coordinates
uint8 currentPhase                                      # current movement-phase obtained from V2X data
uint8 currentLightState                                 # current traffic light state
time timestampCurrentPhase                              # timestamp when the current phase was detected
time remainingDuration                                  # the best predicted remaining time of this phase
uint16 id


# currentPhase
uint8 PHASE_UNAVAILABLE = 0
uint8 PHASE_DARK = 1
uint8 PHASE_STOP_PROCEED = 2
uint8 PHASE_STOP_REMAIN = 3
uint8 PHASE_PRE_MOVEMENET = 4
uint8 PHASE_PERMISSIVE_MOVEMENT = 5
uint8 PHASE_PROTECTED_MOVEMENT = 6
uint8 PHASE_PERMISSIVE_CLEARANCE = 7
uint8 PHASE_PROTECTED_CLEARANCE = 8
uint8 PHASE_CAUTION_CONFLICTING_TRAFFIC = 9

# currentLightState
uint8 STATE_UNAVAILABLE = 0
uint8 STATE_OFF = 1
uint8 STATE_RED = 2
uint8 STATE_YELLOW = 3
uint8 STATE_RED_YELLOW = 4
uint8 STATE_GREEN = 5
uint8 STATE_FLASHING_RED = 6
uint8 STATE_FLASHING_YELLOW = 7
# uint8 RESERVED_1 = 8
# uint8 RESERVED_2 = 9
# uint8 RESERVED_3 = 10
# uint8 RESERVED_4 = 11
# uint8 RESERVED_5 = 12
uint8 STATE_OTHER = 13

