uint32 numPoints                    
geometry_msgs/Point[] imagePoints   
geometry_msgs/Point[] worldPoints
uint8[] type                        # type of border
                                    # maybe add the value to the x-coordinate of imagePoint 

# type
uint8 TYPE_OTHER = 0
uint8 TYPE_CURB = 1
uint8 TYPE_VEHICLE = 2
uint8 TYPE_PERSON = 3
uint8 TYPE_UNDEFINED = 4
