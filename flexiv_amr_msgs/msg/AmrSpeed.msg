std_msgs/Header header

# Actual velocity in the AMR coordinate frame: x/y in m/s, z is yaw rate in rad/s.
geometry_msgs/Vector3 actual_velocity

# Latest velocity received by the AMR controller: x/y in m/s, z is yaw rate in rad/s.
geometry_msgs/Vector3 received_velocity

# Single-steering-wheel angle and tray angle reported by the controller, in rad.
float64 steer
float64 spin

# Received single-steering-wheel angle and tray angular velocity, in rad or rad/s.
float64 received_steer
float64 received_spin

# Multi-steering-wheel angles in model order, in rad.
float64[] steer_angles
float64[] received_steer_angles

bool is_stop
