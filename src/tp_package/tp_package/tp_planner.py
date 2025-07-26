import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import Bool, String
from ackermann_msgs.msg import AckermannDrive
from geometry_msgs.msg import PoseStamped
import math

class TrajectoryPlanner(Node):
    def __init__(self):
        super().__init__('pure_pursuit_trajectory_planner')
        
        # Vehicle state
        self.ego_pose = None
        self.ego_twist = None
        self.path = []
        
        # Control parameters
        self.min_speed = 0.3
        self.max_speed = 0.4
        self.max_steering_angle_deg = 30.0
        self.lookahead_distance = 0.7
        self.wheelbase = 0.50
        self.steering_smoothing_factor = 0.2
        self.smoothed_steering_angle = 0.0
        self.steering_offset_deg = 4.0
        self.end_threshold = 0.3
        
        # Student goal management
        self.student_list = ["First Student", "Second Student", "Third Student", "Destination Reached"]
        self.student_publish_index = 0
        self.ready_to_publish_student = False
        self.student_published_for_this_path = False
        self.was_at_endpoint = False
        
        # Control flags
        self.loop_stopped = False
        self.missing_input_warned = False
        
        # Publishers
        self.drive_pub = self.create_publisher(AckermannDrive, '/ackermann_drive', 10)
        self.student_location_pub = self.create_publisher(String, '/student_location', 10)
        self.student_reached_pub = self.create_publisher(PoseStamped, '/student_reached', 10)
        
        # Subscribers
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(Path, '/path_data', self.path_callback, 10)
        self.create_subscription(Bool, '/obstacle_detected', self.obstacle_callback, 10)
        self.create_subscription(String, '/vehicle_state', self.state_callback, 10)
        
        # Control loop timer (50 Hz)
        self.timer = self.create_timer(0.02, self.control_loop)
        self.get_logger().info("Trajectory Planner initialized.")
    
    def quaternion_to_yaw(self, q):
        """Convert quaternion to yaw angle"""
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)
    
    def get_distance(self, pos1, pos2):
        """Calculate distance between two positions"""
        return math.hypot(pos1.x - pos2.x, pos1.y - pos2.y)
    
    # --- ROS Callbacks ---
    def odom_callback(self, msg):
        """Update ego vehicle pose and twist, adjust to front axle"""
        self.ego_twist = msg.twist.twist
        self.ego_pose = msg.pose.pose
        
        # Adjust to front axle for Pure Pursuit
        wheelbase_half = self.wheelbase / 2.0
        yaw = self.quaternion_to_yaw(self.ego_pose.orientation)
        self.ego_pose.position.x += wheelbase_half * math.cos(yaw)
        self.ego_pose.position.y += wheelbase_half * math.sin(yaw)
    
    def peer_odom_callback(self, msg):
        pass # Not yet implemented
    
    def path_callback(self, msg):
        """Update path and reset student publishing flags"""
        self.path = msg.poses
        self.loop_stopped = False
        self.student_published_for_this_path = False
        self.ready_to_publish_student = True
        self.get_logger().info(f"Received new path with {len(self.path)} waypoints.")
    
    def obstacle_callback(self, msg):
        self.obstacle_detected = msg.data
    
    def state_callback(self, msg):
        self.vehicle_state = msg.data
    
    def prune_path(self):
        """Remove path points behind the vehicle"""
        if not self.path or not self.ego_pose:
            return
        
        ego_pos = self.ego_pose.position
        yaw = self.quaternion_to_yaw(self.ego_pose.orientation)
        
        new_path = []
        for wp in self.path:
            dx = wp.pose.position.x - ego_pos.x
            dy = wp.pose.position.y - ego_pos.y
            x_r = math.cos(-yaw) * dx - math.sin(-yaw) * dy
            
            if x_r > -0.5:
                new_path.append(wp)
        self.path = new_path
    
    def pure_pursuit_steering(self):
        """Calculate steering angle using Pure Pursuit algorithm"""
        if not self.path or not self.ego_pose:
            return 0.0, False
        
        ego_pos = self.ego_pose.position
        yaw = self.quaternion_to_yaw(self.ego_pose.orientation)
        
        # Find lookahead point
        lookahead_point = self.find_lookahead_point(ego_pos, yaw)
        if not lookahead_point:
            return 0.0, False
        
        # Calculate steering
        dx, dy = lookahead_point[0] - ego_pos.x, lookahead_point[1] - ego_pos.y
        x_r = math.cos(-yaw) * dx - math.sin(-yaw) * dy
        y_r = math.sin(-yaw) * dx + math.cos(-yaw) * dy
        
        if x_r <= 0:
            return 0.0, False
        
        ld = math.hypot(x_r, y_r)
        if ld < 1e-3:
            return 0.0, False
        
        curvature = (2 * y_r) / (ld * ld)
        steering_angle_rad = math.atan(self.wheelbase * curvature)
        steering_angle_deg = math.degrees(steering_angle_rad)
        
        # Apply smoothing and saturation
        self.smoothed_steering_angle = ((1 - self.steering_smoothing_factor) * self.smoothed_steering_angle 
                                       + self.steering_smoothing_factor * steering_angle_deg)
        
        saturated = abs(self.smoothed_steering_angle) > self.max_steering_angle_deg
        if saturated:
            self.smoothed_steering_angle = math.copysign(self.max_steering_angle_deg, self.smoothed_steering_angle)
        
        return self.smoothed_steering_angle, saturated
    
    def find_lookahead_point(self, ego_pos, yaw):
        """Find lookahead point for Pure Pursuit"""
        lookahead_point = None
        min_dist_diff = float('inf')
        closest_idx = -1
        
        for i in range(len(self.path) - 1):
            wp1, wp2 = self.path[i].pose.position, self.path[i + 1].pose.position
            d1 = self.get_distance(wp1, ego_pos)
            d2 = self.get_distance(wp2, ego_pos)
            
            # Interpolate if lookahead distance is between waypoints
            if (d1 < self.lookahead_distance <= d2) or (d2 < self.lookahead_distance <= d1):
                ratio = ((self.lookahead_distance - d1) / (d2 - d1)) if (d2 != d1) else 0
                lx = wp1.x + ratio * (wp2.x - wp1.x)
                ly = wp1.y + ratio * (wp2.y - wp1.y)
                return (lx, ly)
            
            # Track closest point as fallback
            dist_diff = abs(self.lookahead_distance - d1)
            if dist_diff < min_dist_diff:
                min_dist_diff = dist_diff
                closest_idx = i
        
        # Use closest or last point if no interpolation found
        if closest_idx >= 0:
            wp = self.path[closest_idx].pose.position
        else:
            wp = self.path[-1].pose.position
        return (wp.x, wp.y)
    
    def publish_velocity(self, speed, steering_angle_deg):
        """Publish Ackermann drive command"""
        if steering_angle_deg != 0:
            steering_angle_deg += math.copysign(self.steering_offset_deg, steering_angle_deg)
        
        steering_angle_deg = max(-self.max_steering_angle_deg, 
                               min(steering_angle_deg, self.max_steering_angle_deg))
        
        drive_msg = AckermannDrive()
        drive_msg.speed = float(speed)
        drive_msg.steering_angle = float(steering_angle_deg)
        self.drive_pub.publish(drive_msg)
    
    def handle_student_goals(self):
        """Manage student pickup/dropoff goals"""
        if not self.path or not self.ego_pose:
            return False
        
        goal_pose = self.path[-1].pose
        ego_pos = self.ego_pose.position
        goal_distance = self.get_distance(ego_pos, goal_pose.position)
        at_endpoint = (goal_distance < self.end_threshold)
        
        if (self.ready_to_publish_student and not self.student_published_for_this_path 
            and self.student_publish_index < len(self.student_list) and at_endpoint and not self.was_at_endpoint):
            
            msg = String()
            msg.data = self.student_list[self.student_publish_index]
            self.student_location_pub.publish(msg)
            print(msg.data)
            
            self.student_publish_index += 1
            self.student_published_for_this_path = True
            self.ready_to_publish_student = False
            
            reached_pose = PoseStamped()
            reached_pose.header.stamp = self.get_clock().now().to_msg()
            reached_pose.header.frame_id = "map"
            reached_pose.pose = goal_pose
            self.student_reached_pub.publish(reached_pose)
        
        self.was_at_endpoint = at_endpoint
        return at_endpoint
    
    def control_loop(self):
        """Main control loop"""
        if self.loop_stopped:
            return
        
        missing = [name for name, value in [('ego_pose', self.ego_pose), ('ego_twist', self.ego_twist), 
                                           ('path_data', self.path)] if not value]
        
        if missing:
            if not self.missing_input_warned:
                self.get_logger().warn(f"Missing inputs: {', '.join(missing)}")
                self.missing_input_warned = True
            return
        else:
            self.missing_input_warned = False
        
        self.prune_path()
        if not self.path:
            self.publish_velocity(0.0, 0.0)
            self.loop_stopped = True
            return
        
        at_endpoint = self.handle_student_goals()
        
        if at_endpoint or self.obstacle_detected or self.vehicle_state.strip() not in ["Driving"]:
            self.publish_velocity(0.0, 0.0)
            if at_endpoint:
                self.loop_stopped = True
            return
        
        steering_angle_deg, saturated = self.pure_pursuit_steering()
        
        abs_steer = min(abs(steering_angle_deg), self.max_steering_angle_deg)
        speed_range = self.max_speed - self.min_speed
        target_v = self.max_speed - (abs_steer / self.max_steering_angle_deg) * speed_range
        target_v = max(self.min_speed, min(target_v, self.max_speed))
        
        self.publish_velocity(target_v, -steering_angle_deg)

def main(args=None):
    rclpy.init(args=args)
    trajectory_planner = TrajectoryPlanner()
    
    try:
        rclpy.spin(trajectory_planner)
    except KeyboardInterrupt:
        trajectory_planner.get_logger().info("Shutting down trajectory planner.")
    finally:
        trajectory_planner.destroy_node()
        rclpy.shutdown

if __name__ == '__main__':
    main()