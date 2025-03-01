import rclpy
from rclpy.node import Node
from interval_analysis_interfaces.msg import Interval as IntervalMsg
from interval_analysis_interfaces.msg import BoxStamped as BoxMsgStamped
from interval_analysis_interfaces.msg import Box as BoxMsg
from interval_analysis_interfaces.msg import BoxListStamped as BoxListStampedMsg
from std_msgs.msg import Header
from builtin_interfaces.msg import Time
from interval_analysis_boat_simu.simu import Simulation
from interval_analysis_boat_simu.boat import Boat
from interval_analysis_boat_simu.buoy import Buoy
from interval_analysis_boat_simu.calcul_tools import *
from interval_analysis_boat_simu.draw import *
import math
import random

class GonioPythonSimuRosNode(Node):
    def __init__(self):
        super().__init__('gonio_python_simu_ros_node')

        """
        SIMULATION
        """
        self.debug = False #print debug in terminal
        # Generate an "∞" trajectory
        t_vals = linspace(0, 2 * pi, 30)  # 100 points for smoothness
        self.trajectory = [[5.0 * sin(t), 5.0 * sin(t) * cos(t)] for t in t_vals]
        #self.trajectory = [[5*t, 0.0] for t in t_vals]
        #self.trajectory = [[2.0,-2.0],[4.0,-2.0],[3.0,2.0],[0.0,2.0]] #list of point to infinitely follow
        self.current_point_to_follow = 0 #index of the current point being followed
        self.goal_success_dist = 1.0
        self.box_save_nb = 100
        self.box_pos_list = []

        #asynchronous sensors publications speeds
        position_rate = 30.0  # Hz
        orientation_rate = 10.0  # Hz
        velocity_rate = 27.0  # Hz
        landmarks_rate = 5.0  # Hz
        simu_rate = max([position_rate,orientation_rate,velocity_rate,landmarks_rate])+1.0 #faster than fastest sensor

        #intervals noise
        self.boat_position_noise_min = 100.0 #0.9
        self.boat_position_noise_max = 100.0 #1.0
        self.boat_orientation_noise_min = 0.0
        self.boat_orientation_noise_max = 0.0
        self.boat_velocity_noise_min = 0.03
        self.boat_velocity_noise_max = 0.05
        self.buoys_angle_noise_min = 0.1
        self.buoys_angle_noise_max = 0.15
        self.buoys_range_noise_min = 0.2
        self.buoys_range_noise_max = 0.25
        self.buoys_position_noise_min = 0.3
        self.buoys_position_noise_max = 0.35

        #parameters
        self.s = 7.0
        self.dt = 1.0/simu_rate
        self.k = 0.5
        self.Ɛ = 2
        #self.num_steps = 1000
        self.record_data = False

        #objects
        self.sea_objects = []
        self.sea_objects.append(Boat(222, -6, 2, 1.5, 0.25, 4, 1.75))
        self.sea_objects.append(Buoy(333, 0, 2, 0, 1, 0, 0))
        self.sea_objects.append(Buoy(334, -1, -2, 0, 1, 0, 0))
        self.sea_objects.append(Buoy(335, 3, 0, 0, 1, 0, 0))
        self.sea_objects.append(Buoy(336, 3, 3, 0, 1, 0, 0))
        self.sea_objects.append(Buoy(337, 4, 2, 0, 1, 0, 0))
        self.sea_objects.append(Buoy(338, 1, 0, 0, 1, 0, 0))
        self.sea_objects.append(Buoy(339, 0, 1, 0, 1, 0, 0))
        self.sea_objects.append(Buoy(330, 2, 4, 0, 1, 0, 0))
        self.sea_objects.append(Buoy(331, -2, -1, 0, 1, 0, 0))
        self.sea_objects.append(Buoy(332, 1, -2, 0, 1, 0, 0))
        self.sea_objects.append(Buoy(341, -6, 2, 0, 1, 0, 0))
        self.sea_objects.append(Buoy(344, -6, -2, 0, 1, 0, 0))
        self.sea_objects.append(Buoy(343, 6, -2, 0, 1, 0, 0))
        self.sea_objects.append(Buoy(342, 6, 2, 0, 1, 0, 0))
        self.sea_objects.append(Buoy(340, -3, 0, 0, 1, 0, 0))
        print('message')

        #visuals
        self.simulation = Simulation(self.sea_objects, self.dt, self.k)
        self.fig, self.ax = init_figure(-self.s, self.s, -self.s, self.s, id="Gonio simulator")

        """
        INITIALISATIONS
        """
        self.t_sim = 0.0

        """
        Dynamic data (Updated by simulation)
        """
        self.box_stamped_position = None # ROS2 BoxMsgStamped
        self.box_stamped_orientation = None # ROS2 BoxMsgStamped
        self.box_stamped_velocity = None # ROS2 BoxMsgStamped
        self.box_list_stamped_landmarks = None # ROS2 BoxListStampedMsg
        self.box_position_contracted = None

        """
        ROS2 Setup
        """
        # Subscriber
        self.box_sub = self.create_subscription(BoxMsgStamped,'/it/contracted/position',self.box_position_callback,10)

        # Publisher
        self.box_stamped_position_pub = self.create_publisher(BoxMsgStamped, '/it/position', 10)
        self.box_stamped_orientation_pub = self.create_publisher(BoxMsgStamped, '/it/orientation', 10)
        self.box_stamped_velocity_pub = self.create_publisher(BoxMsgStamped, '/it/velocity', 10)
        self.box_list_stamped_landmarks_pub = self.create_publisher(BoxListStampedMsg, '/it/landmarks', 10)

        # Separate timers for each message to simulate asynchronous data publications
        self.create_timer(1.0 / position_rate, self.publish_box_stamped_position)
        self.create_timer(1.0 / orientation_rate, self.publish_box_stamped_orientation)
        self.create_timer(1.0 / velocity_rate, self.publish_box_stamped_velocity)
        self.create_timer(1.0 / landmarks_rate, self.publish_box_list_stamped_landmarks)

        #simulation should fatser than sensors rates
        self.create_timer(1.0 / simu_rate, self.simulate)

        self.get_logger().info(f"Simulation started....")

    def publish_box_stamped_position(self):
        if self.box_stamped_position is not None:
            self.box_stamped_position_pub.publish(self.box_stamped_position)
            if self.debug:
                simplified_box = format_box(self.box_stamped_position,3)
                self.get_logger().info(f"\rPublished Position: {simplified_box}"+ 50*" ")

    def publish_box_stamped_orientation(self):
        if self.box_stamped_orientation is not None:
            self.box_stamped_orientation_pub.publish(self.box_stamped_orientation)
            if self.debug:
                simplified_box = format_box(self.box_stamped_orientation,3)
                self.get_logger().info(f"\rPublished Orientation: {simplified_box}"+ 50*" ")

    def publish_box_stamped_velocity(self):
        if self.box_stamped_velocity is not None:
            self.box_stamped_velocity_pub.publish(self.box_stamped_velocity)
            if self.debug:
                simplified_box = format_box(self.box_stamped_velocity,3)
                self.get_logger().info(f"\rPublished Velocity: {simplified_box}"+ 50*" ")

    def publish_box_list_stamped_landmarks(self):
        if self.box_list_stamped_landmarks is not None:
            self.box_list_stamped_landmarks_pub.publish(self.box_list_stamped_landmarks)
            if self.debug:
                debug_txt = "\rPublished Landmarks:" + 50*" "
                for box in self.box_list_stamped_landmarks.boxes:
                    debug_txt = debug_txt + "\n" + str(format_box(box,3))
                self.get_logger().info(f"{debug_txt}")

    def box_position_callback(self, msg):
        self.box_position_contracted = msg


    def simulate(self):
        #simulate and extract observations
        boat = self.sea_objects[0]   # First object should be the boat
        goal = self.trajectory[self.current_point_to_follow]
        goal_dist = sqrt((boat.x-goal[0])**2 + (boat.y-goal[1])**2)
        if goal_dist <= self.goal_success_dist:
            self.current_point_to_follow = (self.current_point_to_follow+1)%len(self.trajectory)
            goal = self.trajectory[self.current_point_to_follow]
        boat.phat = array([[goal[0]],[goal[1]]])
        self.simulation.run(self.record_data, 1, self.ax, self.Ɛ, self.s)

        """
        Convert to intervals and save
        """
        #position
        pos_x, pos_y, pos_z = boat.x, boat.y, 0.0
        pos_x_err = random.uniform(self.boat_position_noise_min, self.boat_position_noise_max)  # Random error between a and b
        pos_y_err = random.uniform(self.boat_position_noise_min, self.boat_position_noise_max)
        pos_z_err = random.uniform(0.0, 0.0) 
        self.box_stamped_position = get_box(float_to_time_msg(self.t_sim),"map",pos_x,pos_y,pos_z,pos_x_err,pos_y_err,pos_z_err,"position")
        #orientation
        angle_x, angle_y, angle_z = 0.0, 0.0, boat.theta
        angle_x_err = random.uniform(0.0, 0.0)  # Random error between a and b
        angle_y_err = random.uniform(0.0, 0.0)
        angle_z_err = random.uniform(self.boat_orientation_noise_min, self.boat_orientation_noise_max) 
        self.box_stamped_orientation = get_box(float_to_time_msg(self.t_sim),"map",angle_x, angle_y, angle_z,angle_x_err,angle_y_err,angle_z_err,"euler_angles")
        #speed
        spd_x, spd_y, spd_z = boat.v, 0.0, 0.0
        spd_x_err = random.uniform(self.boat_velocity_noise_min, self.boat_velocity_noise_max)  # Random error between a and b
        #spd_y_err = random.uniform(self.boat_velocity_noise_min, self.boat_velocity_noise_max)
        spd_y_err = random.uniform(0.0, 0.0) 
        spd_z_err = random.uniform(0.0, 0.0) 
        self.box_stamped_velocity = get_box(float_to_time_msg(self.t_sim),"map",spd_x, spd_y, spd_z,spd_x_err, spd_y_err, spd_z_err,"speed")
        #landmarks
        landmarks_box = [] #Box list with intervals (Angle,range) for each
        
        #self.get_logger().info(f"Interval contracte reçus: {self.box_position_contracted}")
        
        for obj in self.sea_objects[1:]: #we ignore first object, as it is the boat
            if obj.in_area:
                #get position
                mx, my = obj.x, obj.y
                #get angle 
                a = sawtooth(arctan2(my-boat.y,mx-boat.x)-boat.theta)
                #get range
                r = sqrt((mx-boat.x)**2 + (my-boat.y)**2)
                #Convert to intervals
                x_err = random.uniform(self.buoys_position_noise_min, self.buoys_position_noise_max)
                y_err = random.uniform(self.buoys_position_noise_min, self.buoys_position_noise_max)
                a_err = random.uniform(self.buoys_angle_noise_min, self.buoys_angle_noise_max)
                r_err = random.uniform(self.buoys_range_noise_min, self.buoys_range_noise_max)
                x_int = to_interval_msg(mx,x_err,name="x")
                y_int = to_interval_msg(my,y_err,name="y")
                a_int = to_interval_msg(a,a_err,name="angle")
                r_int = to_interval_msg(r,r_err,name="range")
                #Convert and save to Box
                box = BoxMsg()
                box.name = "obj."+str(round(mx,2))+"."+str(round(my,2)) #identified by there position
                box.intervals = [x_int,y_int,a_int,r_int]
                landmarks_box.append(box)
        box_list_msg = BoxListStampedMsg()
        box_list_msg.name = "landmarks"
        box_list_msg.boxes = landmarks_box
        self.box_list_stamped_landmarks = box_list_msg # ROS2 BoxListStampedMsg

        #save
        if self.box_position_contracted is not None:
            self.box_pos_list.append(self.box_position_contracted)
            if(len(self.box_pos_list) > self.box_save_nb):
                self.box_pos_list.pop(0)

        for box_pos in self.box_pos_list:
            draw_box_lines(self.ax, box_pos, color='grey', label=" ", saved=True)

        # Draw waypoints
        waypoints_x = [p[0] for p in self.trajectory]
        waypoints_y = [p[1] for p in self.trajectory]
        self.ax.scatter(waypoints_x, waypoints_y, color='grey', s=1, label='Trajectory')

        # Highlight current goal
        current_goal_x, current_goal_y = boat.phat.flatten()
        self.ax.scatter(current_goal_x, current_goal_y, color='grey', s=20, edgecolors='black', label='Current Goal')

        #Draw raw intervals
        draw_box_lines(self.ax, self.box_stamped_position, color='red', label="raw_boat")
        for box in landmarks_box:
            draw_box_lines(self.ax, box, color='red', label="raw_"+box.name)
            draw_fov_lines(self.ax, boat.x, boat.y, box, color='red', label="raw_"+box.name, off_a = boat.theta)

        #Draw received contracted intervals
        #ros2 topic pub /it/contracted/position interval_analysis_interfaces/msg/Box "{name: 'test_box', intervals: [{name: 'x', start: -1.0, end: 1.0}, {name: 'y', start: -2.0, end: 2.0}]}"
        if self.box_position_contracted is not None:
            draw_box_lines(self.ax, self.box_position_contracted, color='green', label="contracted_boat", pos="down")

        #update time for next iteration
        self.t_sim += self.dt #even if this function is not call at its properrate, we ensure the time iterated is the wanted one


def get_box(timestamp,frame,x,y,z,err_x,err_y,err_z,name="None"):
    box = BoxMsgStamped()
    box.name = name
    box.header = Header(stamp=timestamp, frame_id=frame)
    intervalx = to_interval_msg(x,err_x,"x")
    intervaly = to_interval_msg(y,err_y,"y")
    intervalz = to_interval_msg(z,err_z,"z")
    box.intervals = [intervalx, intervaly, intervalz]
    return box

def float_to_time_msg(t_float):
    # Convert t_sim into a ROS2 Time message
    t_sim_msg = Time()
    t_sim_msg.sec = int(t_float)  # Whole seconds
    t_sim_msg.nanosec = int((t_float - t_sim_msg.sec) * 1e9)  # Remaining nanoseconds
    return t_sim_msg

def to_interval_msg(val,error,name="None"):
    return IntervalMsg(name=name, start=val-abs(error), end=val+abs(error))

def format_box(box_msg,precision):
    """Format a BoxMsg or BoxStampedMsg as a list of [start, end] pairs."""
    return [[round(interval.start,precision), round(interval.end,precision)] for interval in box_msg.intervals]

def draw_box_lines(ax, box_msg, color='red', label=None, pos="top", backup=None, saved = False):
    """
    Draws a square box using simple lines based on a BoxMsg.

    Parameters:
        ax: Matplotlib axis to draw on.
        box_msg: A BoxMsg containing position interval data.
        color: Color of the box lines.
        label: Optional label for the box.
    """
    # Extract intervals
    x_interval = [interval for interval in box_msg.intervals if interval.name == "x"][0]
    y_interval = [interval for interval in box_msg.intervals if interval.name == "y"][0]

    # Get box boundaries
    x_min, x_max = x_interval.start, x_interval.end
    y_min, y_max = y_interval.start, y_interval.end

    finite_vals = True
    if np.isinf([x_min, x_max,y_min, y_max]).any() or np.isnan([x_min, x_max,y_min, y_max]).any():
        x_min, x_max,y_min, y_max = -0.1, 0.1, -0.1, 0.1
        finite_vals = False

    # Define the four corners
    corners = [
        (x_min, y_min),
        (x_max, y_min),
        (x_max, y_max),
        (x_min, y_max),
        (x_min, y_min)  # Close the box
    ]

    # Extract x and y coordinates
    x_vals, y_vals = zip(*corners)

    # Draw the box using simple lines
    linestyle = '-' # Solid line
    if not finite_vals:  # Check if any value in x_vals is infinite
        linestyle=(0, (5, 5))  # Dashed pattern "- - - -"
    if not saved:
        ax.plot(x_vals, y_vals, color=color, linestyle=linestyle, linewidth=1)
    else:
        ax.plot(x_vals, y_vals, color=color, linestyle=linestyle, linewidth=1,alpha=0.2)

    # Draw the center point
    if not saved:
        center_x, center_y = (x_min + x_max) / 2, (y_min + y_max) / 2
        ax.plot(center_x, center_y, color=color, marker='x')

    # Add label
    if label and not saved:
        #width = abs(x_max-x_min)
        height = abs(y_max-y_min)
        x_text = center_x
        y_text = center_y
        if pos == "top":
            y_text += (height/2)+0.30
        elif pos == "down":
            y_text -= (height/2)+0.30
        txt = label
        if not finite_vals:
            txt = "inf"
        ax.text(x_text, y_text, txt, fontsize=10, ha='center', va='center', color=color)

def draw_fov_lines(ax, robot_x, robot_y, box_msg, color='red', linewidth=1.0, label=None, off_a = 0.0):
    """
    Draws two cropped arcs using lines and connects them with radial lines.
    
    Parameters:
    - ax: Matplotlib axis
    - robot_x, robot_y: Position of the robot
    - angle_min, angle_max: Field of view limits (radians)
    - range_min, range_max: Min and max range values
    - color: Line color
    - linewidth: Line thickness
    """
    angle_interval = [interval for interval in box_msg.intervals if interval.name == "angle"][0]
    range_interval = [interval for interval in box_msg.intervals if interval.name == "range"][0]
    angle_min, angle_max = angle_interval.start+off_a, angle_interval.end+off_a
    range_min, range_max = range_interval.start, range_interval.end

    # Generate points for the inner arc (minimum range)
    theta_inner = np.linspace(angle_min, angle_max, 30)  # Smooth arc
    inner_x = robot_x + range_min * np.cos(theta_inner)
    inner_y = robot_y + range_min * np.sin(theta_inner)
    
    # Generate points for the outer arc (maximum range)
    theta_outer = np.linspace(angle_min, angle_max, 30)  # Smooth arc
    outer_x = robot_x + range_max * np.cos(theta_outer)
    outer_y = robot_y + range_max * np.sin(theta_outer)

    # Draw the inner arc
    ax.plot(inner_x, inner_y, color=color, linewidth=linewidth)

    # Draw the outer arc
    ax.plot(outer_x, outer_y, color=color, linewidth=linewidth)

    # Compute endpoints for the FOV boundaries
    x1_min, y1_min = robot_x + range_min * np.cos(angle_min), robot_y + range_min * np.sin(angle_min)
    x1_max, y1_max = robot_x + range_max * np.cos(angle_min), robot_y + range_max * np.sin(angle_min)
    x2_min, y2_min = robot_x + range_min * np.cos(angle_max), robot_y + range_min * np.sin(angle_max)
    x2_max, y2_max = robot_x + range_max * np.cos(angle_max), robot_y + range_max * np.sin(angle_max)

    # Draw the two radial lines to connect the arcs
    ax.plot([x1_min, x1_max], [y1_min, y1_max], color=color, linewidth=linewidth)  # Left FOV boundary
    ax.plot([x2_min, x2_max], [y2_min, y2_max], color=color, linewidth=linewidth)  # Right FOV boundary

    if label:
        x_text = (min([x1_min,x1_max,x2_min,x2_max])+max([x1_min,x1_max,x2_min,x2_max]))/2
        y_text = min([y1_min,y1_max,y2_min,y2_max])-0.30
        ax.text(x_text, y_text, label, fontsize=10, ha='center', va='center', color=color)

def main(args=None):
    rclpy.init(args=args)
    gonio_python_simu_ros_node = GonioPythonSimuRosNode()
    rclpy.spin(gonio_python_simu_ros_node)
    gonio_python_simu_ros_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
