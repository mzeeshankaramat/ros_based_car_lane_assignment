
# ROS Header information containg
# uint32 seq, time stamp, string frame_id
std_msgs/Header header                    


# number of passengers
uint16 passenger_count
# timestamp of passenger count in seconds since jan. 1. 1970
uint64 passenger_count_epoch_seconds
# nanosecond part of timestamp 
uint32 passenger_count_epoch_nanoseconds

# number of passengers sitting
uint16 passenger_sitting
# timestamp of passenger sitting in seconds since jan. 1. 1970
uint64 passenger_sitting_epoch_seconds
# nanosecond part of timestamp 
uint32 passenger_sitting_epoch_nanoseconds

# number of passengers standing
uint16 passenger_standing
# timestamp of passenger standing in seconds since jan. 1. 1970
uint64 passenger_standing_epoch_seconds
# nanosecond part of timestamp 
uint32 passenger_standing_epoch_nanoseconds

