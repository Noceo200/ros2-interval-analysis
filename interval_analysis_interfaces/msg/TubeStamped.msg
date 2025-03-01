# TubeStamped.msg
# ROS2 message definition for a collection of boxes with temporal metadata, with a header for timestamp and frame reference

std_msgs/Header header # Standard header for timestamps and frame reference
string name # Optionnal
Box[] boxes # List of boxes
Interval t_domain # Time domain start and end (e.g., for time-series or interval analysis)
float64 dt # Time step or interval duration
