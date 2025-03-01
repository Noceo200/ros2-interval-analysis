# Box.msg
# ROS2 message definition for a box (Interval vector) represented by a list of intervals with a header for timestamp and frame reference

std_msgs/Header header # Standard header for timestamps and frame reference
string name # Optionnal
Interval[] intervals # List of intervals defining the box (e.g., spatial bounds, time intervals)
