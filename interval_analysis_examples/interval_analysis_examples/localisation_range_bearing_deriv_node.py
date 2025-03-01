import rclpy
from rclpy.node import Node
from codac import *
from interval_analysis_interfaces.msg import Interval as IntervalMsg
from interval_analysis_interfaces.msg import BoxStamped as BoxMsgStamped
from interval_analysis_interfaces.msg import BoxListStamped as BoxListStampedMsg
from std_msgs.msg import Header
from builtin_interfaces.msg import Time

class ContractorNode(Node):
    def __init__(self):
        super().__init__('contractor_node')
        """
        PARAMETERS
        """
        self.debug = False
        self.run_rate = 10.0
        self.max_samples_saved = 1000 #if 0, we keep all
        self.dt_tube = 5/self.run_rate

        """
        Varaiables
        """
        self.x_interval_list = []
        self.y_interval_list = []
        self.vx_interval_list = []
        self.vy_interval_list = []
        self.t_list = []
        self.x_tube_vector,self.v_tube_vector = None, None

        """
        Messages (Updated by subscriptions)
        """
        self.box_stamped_position = None # ROS2 BoxMsgStamped
        self.box_stamped_orientation = None # ROS2 BoxMsgStamped
        self.box_stamped_velocity = None # ROS2 BoxMsgStamped
        self.box_list_stamped_landmarks = None # ROS2 BoxListStampedMsg
        self.box_position_contracted = None # ROS2 BoxMsgStamped

        """
        CONTRACTORS
        """
        # Gonio Contractors
        self.ctc_polar = CtcPolar()
        self.ctc_2 = CtcFunction(Function("x[3]", "l[4]", "d1", "l[0]-x[0]-d1"))
        self.ctc_3 = CtcFunction(Function("x[3]", "l[4]", "d2", "l[1]-x[1]-d2"))
        self.ctc_4 = CtcFunction(Function("x[3]", "l[4]", "theta", "x[2]+l[2]-theta"))

        """
        ROS2
        """
        self.box_stamped_position_raw_sub = self.create_subscription(BoxMsgStamped,'/it/position',self.box_position_raw_callback,10)
        self.box_stamped_orientation_raw_sub = self.create_subscription(BoxMsgStamped,'/it/orientation',self.box_orientation_raw_callback,10)
        self.box_stamped_velocity_raw_sub = self.create_subscription(BoxMsgStamped,'/it/velocity',self.box_velocity_raw_callback,10)
        self.box_stamped_landmarks_raw_sub = self.create_subscription(BoxListStampedMsg,'/it/landmarks',self.box_landmarks_raw_callback,10)
        self.publisher_contracted_position = self.create_publisher(BoxMsgStamped, '/it/contracted/position', 10)
        #main loop
        self.create_timer(1.0 / self.run_rate, self.run)

    def box_position_raw_callback(self, msg):
        self.box_stamped_position = msg
        if self.debug:
            self.get_logger().info(f"Received Position: {str(format_box(msg,3))}")

    def box_orientation_raw_callback(self, msg):
        self.box_stamped_orientation = msg
        if self.debug:
            self.get_logger().info(f"Received Orientation: {str(format_box(msg,3))}")

    def box_velocity_raw_callback(self, msg):
        self.box_stamped_velocity = msg
        if self.debug:
            self.get_logger().info(f"Received Velocity: {str(format_box(msg,3))}")

    def box_landmarks_raw_callback(self, msg):
        self.box_list_stamped_landmarks = msg
        if self.debug:
            debug_txt = "Received Landmarks: "
            for box in msg.boxes:
                debug_txt += f"\n{str(format_box(box,3))}"
            self.get_logger().info(debug_txt)

    def contract_ocean_range(self,x_state,landmarks):
        """
        x = IntervalVector[x,y,tetha]
        landmarks = [IntervalVector[x,y,tetha,range],...]
        """
        def add_landmark_to_contractor_network(cn,x_state,landmark):
            # Intermediate variables
            m = landmark.subvector(0,1) #x,y
            y = landmark.subvector(2,3) #a,r
            theta = y[0] + x_state[2]
            d1 = m[0] - x_state[0]
            d2 = m[1] - x_state[1]
            #add contractors
            cn.add(self.ctc_polar, [d1, d2, y[1], theta])
            cn.add(self.ctc_2, [x_state, landmark, d1])
            cn.add(self.ctc_3, [x_state, landmark, d2])
            cn.add(self.ctc_4, [x_state, landmark, theta])
        if len(landmarks) > 0:
            cn = ContractorNetwork()
            for landmark in landmarks:
                add_landmark_to_contractor_network(cn,x_state,landmark)
            cn.contract()
        return x_state
    
    def publish_position_from_state(self,x_state):
        err_x = (x_state[0].ub() - x_state[0].lb())/2.0
        err_y = (x_state[1].ub() - x_state[1].lb())/2.0
        err_z = 0.0
        mx = x_state[0].lb() + err_x
        my = x_state[1].lb() + err_y
        mz = 0.0
        self.publisher_contracted_position.publish(get_box(self.box_stamped_position.header.stamp,self.box_stamped_position.header.frame_id,mx,my,mz,err_x,err_y,err_z,name="contracted_boat"))
        
    def contract_deriv(self,x_tube_vector,v_tube_vector,x_state):
        ctc.deriv.contract(x_tube_vector,v_tube_vector)
        x_int = x_tube_vector[0].last_slice().output_gate()
        y_int = x_tube_vector[1].last_slice().output_gate()
        x_state = IntervalVector([x_int,y_int,x_state[2]])
        return x_state
    
    def save_list_vector(self,x_box,v_box,t_now):
        self.x_interval_list.append(x_box[0])
        self.y_interval_list.append(x_box[1])
        self.vx_interval_list.append(v_box[0])
        self.vy_interval_list.append(v_box[1])
        self.t_list.append(t_now)
        if len(self.t_list) > self.max_samples_saved and self.max_samples_saved is not 0:
            self.x_interval_list.pop(0)
            self.y_interval_list.pop(0)
            self.vx_interval_list.pop(0)
            self.vy_interval_list.pop(0)
            self.t_list.pop(0)

    def update_tube_vectors(self):
        traj_x_lb, traj_x_ub = get_trajs_from_interval_list(self.t_list,self.x_interval_list)
        x_tube = get_tube_from_trajs(self.dt_tube,traj_x_lb,traj_x_ub)
        traj_y_lb, traj_y_ub = get_trajs_from_interval_list(self.t_list,self.y_interval_list)
        y_tube = get_tube_from_trajs(self.dt_tube,traj_y_lb,traj_y_ub)
        traj_vx_lb, traj_vx_ub = get_trajs_from_interval_list(self.t_list,self.vx_interval_list)
        vx_tube = get_tube_from_trajs(self.dt_tube,traj_vx_lb,traj_vx_ub)
        traj_vy_lb, traj_vy_ub = get_trajs_from_interval_list(self.t_list,self.vy_interval_list)
        vy_tube = get_tube_from_trajs(self.dt_tube,traj_vy_lb,traj_vy_ub)
        self.x_tube_vector = TubeVector([x_tube,y_tube])
        self.v_tube_vector = TubeVector([vx_tube,vy_tube])

    def run(self):
        if self.box_stamped_position is not None and self.box_stamped_orientation is not None and self.box_list_stamped_landmarks is not None and self.box_stamped_velocity is not None:
            #Get Interval objects for robot
            pos_interval_vector = box_msg_to_interval_vector(self.box_stamped_position)
            orientation_interval_vector = box_msg_to_interval_vector(self.box_stamped_orientation)
            x = IntervalVector([pos_interval_vector[0],pos_interval_vector[1],orientation_interval_vector[2]])
            #Get Intevral objects for landmarks
            landmarks = []
            for landmark_msg in self.box_list_stamped_landmarks.boxes:
                landmarks.append(box_msg_to_interval_vector(landmark_msg))
            #contract and publish
            """
            And any contraction strategy here
            """
            if len(self.t_list) > 20:
                self.update_tube_vectors()
                self.x_tube_vector.set((-6.0,2.0),0)
                contracted_state_deriv = self.contract_deriv(self.x_tube_vector,self.v_tube_vector,x)
                self.publish_position_from_state(contracted_state_deriv)
            self.save_list_vector(x.subvector(0,1),convert_to_world(box_msg_to_interval_vector(self.box_stamped_velocity),x[2]).subvector(0,1),time_msg_to_float(self.box_stamped_position.header.stamp))
            
            contracted_state_gonio = self.contract_ocean_range(x,landmarks) #x,y,tetha
            self.publish_position_from_state(contracted_state_gonio)
            
            self.save_list_vector(contracted_state_gonio.subvector(0,1),convert_to_world(box_msg_to_interval_vector(self.box_stamped_velocity),x[2]).subvector(0,1),time_msg_to_float(self.box_stamped_position.header.stamp))
            if len(self.t_list) > 2:
                self.update_tube_vectors()
                contracted_state_deriv = self.contract_deriv(self.x_tube_vector,self.v_tube_vector,contracted_state_gonio)
                self.publish_position_from_state(contracted_state_deriv)
            else:
                self.publish_position_from_state(contracted_state_gonio)

def convert_to_world(speed_interval_vector,bot_heading):
    v_norm = speed_interval_vector[0] #interval
    x_speed = v_norm*cos(bot_heading) #interval 
    y_speed = v_norm*sin(bot_heading) #interval
    return IntervalVector([x_speed,y_speed,bot_heading])

def get_tube_from_trajs(dt,traj_lb,traj_ub):
    tube = Tube(traj_lb,dt)
    tube |= traj_ub
    return tube

def get_trajs_from_interval_list(t_list,interval_list):
    traj_lb = Trajectory(t_list,[interval.lb() for interval in interval_list])
    traj_ub = Trajectory(t_list,[interval.ub() for interval in interval_list])
    return traj_lb,traj_ub

def box_msg_to_interval_vector(msg) :
    interval_list = []
    for interval_msg in msg.intervals:
        interval = interval_msg_to_interval(interval_msg)
        interval_list.append(interval)
    interval_vector = IntervalVector(interval_list)
    return interval_vector

def interval_msg_to_interval(msg):
    interval = Interval(msg.start,msg.end)
    return interval

def format_box(box_msg,precision):
    """Format a BoxMsg or BoxStampedMsg as a list of [start, end] pairs."""
    return [[round(interval.start,precision), round(interval.end,precision)] for interval in box_msg.intervals]

def get_box(timestamp,frame,x,y,z,err_x,err_y,err_z,name="None"):
    box = BoxMsgStamped()
    box.name = name
    box.header = Header(stamp=timestamp, frame_id=frame)
    intervalx = to_interval_msg(x,err_x,"x")
    intervaly = to_interval_msg(y,err_y,"y")
    intervalz = to_interval_msg(z,err_z,"z")
    box.intervals = [intervalx, intervaly, intervalz]
    return box

def to_interval_msg(val,error,name="None"):
    return IntervalMsg(name=name, start=val-abs(error), end=val+abs(error))

def float_to_time_msg(t_float):
    # Convert t_sim into a ROS2 Time message
    t_sim_msg = Time()
    t_sim_msg.sec = int(t_float)  # Whole seconds
    t_sim_msg.nanosec = int((t_float - t_sim_msg.sec) * 1e9)  # Remaining nanoseconds
    return t_sim_msg

def time_msg_to_float(time_msg):
    # Convert a ROS2 Time message into a float
    return time_msg.sec + time_msg.nanosec * 1e-9

def main(args=None):
    rclpy.init(args=args)
    contractor_node = ContractorNode()
    rclpy.spin(contractor_node)
    contractor_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()