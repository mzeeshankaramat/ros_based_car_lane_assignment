# [units]

# [sec], [nsec]
time creation_timestamp           # timestamp of scanning start time of the object (unix time)

# []
uint16 id                           # object ID

# []  
uint16 age                          # in how many scans was the object already seen


# [] -> range [0, 1, ..., 12] 
# -> enum (see below)
uint8 classification                # classifiation of the object: (Car,Bus,Truck, Pedestrian...)

# [] -> range [0 - 1]  
float32 classification_certainty    # certainty about the right classification

float32 existence_confidence         # existence confidence of object



# [] -> range [0 - 1]  
float32 within_path_probability     # probability of object to be within ego path

# [sec]
float64 time_to_collision           # ttc in s, time until the object will collide with the vehicle 

# []                                      
bool will_collide                   # whether the object will collide wiht the vehicle


# [] -> range [0, 1, 2, 3] 
# -> enum (see below)
uint8 lane_assignment               # on which lane the object is relative to the the vehicle (left, ego, right)

# [] -> range [0, 1, ... 6] 
# -> enum (see below)
uint8 movement_state               # movement status (dynamic,movable...)

# [] -> range [0, 1, ... 9] 
# -> enum (see below)                                        
uint8 moving_direction              # the direction in which the the vehicle moves (same direction...)


# [m] (for each point entry)
geometry_msgs/Point size            # size along x,y,z in m

# [m] (for each point entry)                                       
geometry_msgs/Point position        # horizontal and vertical center point of the object

# [rad]                                       
float32 orientation_rad             # rotation of the object in rad

# [deg]                                       
float32 orientation_middle_deg      # angular position of the object in degree in relation to the camera-center
float32 orientation_left_deg
float32 orientation_right_deg 

# [m/s] (for each point entry) 
geometry_msgs/Point rel_speed       # relative speed along x,y,z

# [m/s] (for each point entry)                                        
geometry_msgs/Point abs_speed       # absolute speed along x,y,z

# [m^2/s] (for each point entry)                                        
geometry_msgs/Point rel_acc         # relative acceleration x,y,z

# [m] (for each point entry)
geometry_msgs/Point[] contour_point_list     # list of contour points of an object in m

box cv_bounding_box                     # bounding box in image space
geometry_msgs/Vector3 cv_direction     # moving direction of object in image space
geometry_msgs/Vector3 cv_image_velocity # velocity of object in image space
float32 cv_scaleFactor                 # scale factor of boundingbox from the previous frame
bool cv_brake_light                  # indicates whether braking light is on
bool cv_turning_indicator_left        # indicates whether turning left light is on
bool cv_turning_indicator_right    # indicates whether turning right light is on

# classification
uint8 CL_UNKNOWN = 0
uint8 CL_UNKNOWN_SMALL = 1
uint8 CL_UNKNOWN_BIG = 2
uint8 CL_CAR = 3
uint8 CL_BUS = 4
uint8 CL_TRUCK = 5
uint8 CL_TWO_WHEEL = 6
uint8 CL_OVERDRIVABLE = 7
uint8 CL_UNDERDRIVABLE = 8
uint8 CL_TRAFFIC_LIGHT = 9
uint8 CL_EVENT = 10
uint8 CL_PEDESTRIAN = 11
uint8 CL_OTHER = 12

# lane_assignment
uint8 LA_UNKNOWN = 0
uint8 LA_LEFT_LANE = 1
uint8 LA_EGO_LANE = 2
uint8 LA_RIGHT_LANE = 3


# movement_status
uint8 MS_UNKNOWN = 0
uint8 MS_PROBABLE_STATIC = 1
uint8 MS_STATIC_OBJ = 2
uint8 MS_PROBABLE_DYNAMIC = 3
uint8 MS_DYNAMIC = 4
uint8 MS_MOVABLE = 5
uint8 MS_NUMBER_OF_ENTRIES = 6

# moving_direction
uint8 MD_UNKNOWN = 0
uint8 MD_PROBABLE_SAME_DIRECTION = 1
uint8 MD_SAME_DIRECTION = 2
uint8 MD_PROBABLE_OPPOSITE_DIRECTION = 3
uint8 MD_OPPOSITE_DIRECTION = 4
uint8 MD_PROBABLE_LEFT_TO_RIGHT_CROSSING = 5
uint8 MD_LEFT_TO_RIGHT_CROSSING = 6
uint8 MD_PROBABLE_RIGHT_TO_LEFT_CROSSING = 7
uint8 MD_RIGHT_TO_LEFT_CROSSING = 8
uint8 MD_NUMBER_OF_ENTRIES = 9
