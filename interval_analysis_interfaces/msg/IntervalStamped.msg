# Interval.msg
# ROS2 message definition for interval analysis with a header for timestamp and frame reference

std_msgs/Header header # Standard header for timestamps and frame reference
string name # Optionnal
float64 start # Start of the interval (e.g., time, range lower bound)
float64 end # End of the interval (e.g., time, range upper bound)
