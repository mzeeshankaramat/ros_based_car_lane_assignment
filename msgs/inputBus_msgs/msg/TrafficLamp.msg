uint8 lightState                # state as detected by camera
uint32 lightStateProbability
uint8 pictogram
uint32 pictogramProbability
int32 lightDirection            # angle of traffic light signal (-180 to 180 degrees w.r.t. driving direction)
inputBus_msgs/box boundingBox
uint16 id
float32 existence_confidence 

# lightState
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

# pictogram
uint8 PIC_UNAVAILABLE = 0
uint8 PIC_ROUND = 1
uint8 PIC_ARROW_LEFT = 2
uint8 PIC_ARROW_RIGHT = 3
uint8 PIC_ARROW_STRAIGHT = 4
uint8 PIC_ARROW_STRAIGHT_RIGHT = 5
uint8 PIC_ARROW_STRAIGHT_LEFT = 6
uint8 PIC_PEDESTRIAN = 7
uint8 PIC_CYCLIST = 8
# uint8 RESERVED_1 = 9
# uint8 RESERVED_2 = 10
# uint8 RESERVED_3 = 11
# uint8 RESERVED_4 = 12
# uint8 RESERVED_5 = 13
uint8 PIC_OTHER = 14
