# BoxStampedList.msg
# ROS2 message definition for a list of box (Interval vector)

std_msgs/Header header # Standard header for timestamps and frame reference
string name 
Box[] boxes # List of boxes
