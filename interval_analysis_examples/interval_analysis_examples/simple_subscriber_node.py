import rclpy
from rclpy.node import Node
from interval_analysis_interfaces.msg import Interval as IntervalMsg
from interval_analysis_interfaces.msg import Box as BoxMsg
from interval_analysis_interfaces.msg import Tube as TubeMsg
from interval_analysis_interfaces.msg import BoxListStamped as BoxListStampedMsg
from std_msgs.msg import Header

class SubscriberNode(Node):
    def __init__(self):
        super().__init__('subscriber_node')

        # Subscribers for each message type
        self.interval_sub = self.create_subscription(
            IntervalMsg,
            'interval_topic',
            self.interval_callback,
            10
        )
        self.box_sub = self.create_subscription(
            BoxMsg,
            'box_topic',
            self.box_callback,
            10
        )
        self.tube_sub = self.create_subscription(
            TubeMsg,
            'tube_topic',
            self.tube_callback,
            10
        )
        self.box_list_stamped_sub = self.create_subscription(
            BoxListStampedMsg,
            'box_stamped_list_topic',
            self.box_list_stamped_callback,
            10
        )

    def interval_callback(self, msg):
        # Logging the received Interval message
        self.get_logger().info(f"Received Interval: Name = {msg.name}, Start = {msg.start}, End = {msg.end}")
        
        # Check for infinite bounds
        if msg.start == -float('inf'):
            self.get_logger().info("This interval starts at negative infinity.")
        if msg.end == float('inf'):
            self.get_logger().info("This interval ends at positive infinity.")

    def box_callback(self, msg):
        # Logging the received Box message
        self.get_logger().info(f"Received Box: Name = {msg.name}")
        
        # Process each interval inside the Box
        for interval in msg.intervals:
            self.get_logger().info(f"  Interval: Name = {interval.name}, Start = {interval.start}, End = {interval.end}")
            
            # Check for infinite bounds in the intervals
            if interval.start == -float('inf'):
                self.get_logger().info(f"  {interval.name} starts at negative infinity.")
            if interval.end == float('inf'):
                self.get_logger().info(f"  {interval.name} ends at positive infinity.")

    def tube_callback(self, msg):
        # Logging the received Tube message
        self.get_logger().info(f"Received Tube: Name = {msg.name}, Time Domain: Start = {msg.t_domain.start}, End = {msg.t_domain.end}, Time Step = {msg.dt}")
        
        # Process each box inside the Tube
        for box in msg.boxes:
            self.get_logger().info(f"  Box: Name = {box.name}")
            
            # Process each interval inside the Box
            for interval in box.intervals:
                self.get_logger().info(f"    Interval: Name = {interval.name}, Start = {interval.start}, End = {interval.end}")
                
                # Check for infinite bounds
                if interval.start == -float('inf'):
                    self.get_logger().info(f"    {interval.name} starts at negative infinity.")
                if interval.end == float('inf'):
                    self.get_logger().info(f"    {interval.name} ends at positive infinity.")

    def box_list_stamped_callback(self, msg):
        self.get_logger().info(f"Received BoxListStamped: Name = {msg.name}")
        self.get_logger().info(f"  - Timestamp: {msg.header.stamp.sec}.{msg.header.stamp.nanosec} | Frame ID: {msg.header.frame_id}")
        for box in msg.boxes:
            self.get_logger().info(f"    Box Name: {box.name}")
            for interval in box.intervals:
                self.get_logger().info(f"      Interval: Name = {interval.name}, Start = {interval.start}, End = {interval.end}")
                if interval.start == -float('inf'):
                    self.get_logger().info(f"      {interval.name} starts at negative infinity.")
                if interval.end == float('inf'):
                    self.get_logger().info(f"      {interval.name} ends at positive infinity.")


def main(args=None):
    rclpy.init(args=args)
    subscriber_node = SubscriberNode()
    rclpy.spin(subscriber_node)
    subscriber_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
