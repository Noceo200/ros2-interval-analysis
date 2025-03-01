# Interval.msg
# ROS2 message definition for interval analysis without a header for timestamp and frame reference

string name # Optionnal
float64 start # Start of the interval (e.g., time, range lower bound)
float64 end # End of the interval (e.g., time, range upper bound)
