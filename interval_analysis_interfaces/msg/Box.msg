# Box.msg
# ROS2 message definition for a box (Interval vector) represented by a list of intervals without a header for timestamp and frame reference

string name # Optionnal
Interval[] intervals # List of intervals defining the box (e.g., spatial bounds, time intervals)
