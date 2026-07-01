std_msgs/Header header

# Battery level in [0, 1].
float64 battery_level

# Battery temperature in degrees Celsius.
float64 battery_temp

# Battery voltage in V and current in A.
float64 voltage
float64 current

# Maximum allowed charging voltage/current. A value of -1 means unsupported by the battery.
float64 max_charge_voltage
float64 max_charge_current

bool charging
bool manual_charge
bool auto_charge

int32 battery_cycle
string battery_user_data
string extra
