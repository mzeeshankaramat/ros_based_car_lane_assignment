# ROS Header information containg
# uint32 seq, time stamp, string frame_id
std_msgs/Header header

# max distance between lanes
float64 ego_left_dist_max
float64 ego_right_dist_max

# area between lanes
float64 ego_left_area
float64 ego_right_area

# mean distance between lanes
float64 ego_left_dist_mean
float64 ego_right_dist_mean

# variance of lane distances
float64 ego_left_dist_variance
float64 ego_right_dist_variance

# Normalized area
float64 ego_left_area_normalized
float64 ego_right_area_normalized