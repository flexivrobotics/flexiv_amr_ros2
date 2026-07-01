std_msgs/Header header

# AMR pose in the current map/world frame.
float64 x
float64 y
float64 angle

# Localization confidence in [0, 1].
float64 confidence

# SEER localization method code.
int32 localization_method

string current_station
string last_station
