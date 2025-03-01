import rclpy
from rclpy.node import Node
from interval_analysis_interfaces.msg import Interval as IntervalMsg
from interval_analysis_interfaces.msg import Box as BoxMsg
from interval_analysis_interfaces.msg import Tube as TubeMsg
from interval_analysis_interfaces.msg import BoxListStamped as BoxListStampedMsg
from std_msgs.msg import Header

class PublisherNode(Node):
    def __init__(self):
        super().__init__('publisher_node')

        # Publishers for each message type
        self.interval_pub = self.create_publisher(IntervalMsg, 'interval_topic', 10)
        self.box_pub = self.create_publisher(BoxMsg, 'box_topic', 10)
        self.tube_pub = self.create_publisher(TubeMsg, 'tube_topic', 10)
        self.box_list_stamped_pub = self.create_publisher(BoxListStampedMsg, 'box_stamped_list_topic', 10)

        # Separate timers for each message type
        self.create_timer(1.0, self.publish_interval)  # Publish Interval every 1 second
        self.create_timer(1.0, self.publish_box)       # Publish Box every 2 seconds
        self.create_timer(1.0, self.publish_tube)      # Publish Tube every 3 seconds
        self.create_timer(4.0, self.publish_box_list_stamped)  # Publish BoxStampedList every 4 seconds

    def publish_interval(self):
        interval = IntervalMsg()
        interval.name = "Example Interval"
        interval.start = -float('inf') # Infinite bound
        interval.end = 10.0
        self.interval_pub.publish(interval)
        self.get_logger().info(f'\nPublished Interval: {interval}\n')

    def publish_box(self):
        # Create three different intervals for the first box
        interval1 = IntervalMsg(name="Interval1", start=0.0, end=5.0)
        interval2 = IntervalMsg(name="Interval2", start=-5.0, end=10.0)
        interval3 = IntervalMsg(name="Interval3", start=-float('inf'), end=float('inf'))  # Infinite bounds

        # Create the first box with these three intervals
        box = BoxMsg()
        box.name = "Example Box or IntervalVector with 3 Dimensions"
        box.intervals = [interval1, interval2, interval3]
        
        # Publish the box or IntervalVector
        self.box_pub.publish(box)
        
        self.get_logger().info(f'\nPublished Box: {box}\n')

    def publish_tube(self):
        # Create the tube containing two boxes (Box1 and Box2)
        tube = TubeMsg()
        tube.name = "Example Tube"
        
        # Create the time domain for the tube (interval representing time span)
        t_domain = IntervalMsg(name="Time Domain", start=0.0, end=2.0)
        
        # Box1 and Box2 are the same but represent different times (t1 and t2)
        box1 = BoxMsg(name="Box1 at t1", intervals=[IntervalMsg(name="Interval1", start=0.0, end=5.0),
                                                  IntervalMsg(name="Interval2", start=5.0, end=10.0),
                                                  IntervalMsg(name="Interval3", start=10.0, end=15.0)])
        box2 = BoxMsg(name="Box2 at t2", intervals=[IntervalMsg(name="Interval1", start=0.0, end=5.0),
                                                  IntervalMsg(name="Interval2", start=8.0, end=10.0),
                                                  IntervalMsg(name="Interval3", start=10.0, end=10.0)])
        
        tube.boxes = [box1, box2]
        tube.t_domain = t_domain
        tube.dt = 1.0  # Time step between the boxes (e.g., 1 second)
        
        # Publish the tube
        self.tube_pub.publish(tube)
        self.get_logger().info(f'\nPublished Tube: {tube}\n')

    def publish_box_list_stamped(self):
        box_list_stamped = BoxListStampedMsg()
        box_list_stamped.name = "Example BoxStampedList"

        # Create two stamped boxes
        box1 = BoxMsg(name="Box1 at t1", intervals=[IntervalMsg(name="Interval1", start=0.0, end=5.0),
                                                  IntervalMsg(name="Interval3", start=10.0, end=15.0)])
        box2 = BoxMsg(name="Box2 at t2", intervals=[IntervalMsg(name="Interval1", start=0.0, end=5.0),
                                                  IntervalMsg(name="Interval2", start=8.0, end=10.0),
                                                  IntervalMsg(name="Interval3", start=10.0, end=10.0)])

        box_list_stamped.header = Header(stamp=self.get_clock().now().to_msg(), frame_id="world")
        box_list_stamped.boxes = [box1, box2]

        self.box_list_stamped_pub.publish(box_list_stamped)
        self.get_logger().info(f'\nPublished BoxStampedList: {box_list_stamped}\n')


def main(args=None):
    rclpy.init(args=args)
    publisher_node = PublisherNode()
    rclpy.spin(publisher_node)
    publisher_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()