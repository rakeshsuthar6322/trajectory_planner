import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import Bool, String
from ackermann_msgs.msg import AckermannDrive
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
import math
import numpy as np
from enum import Enum

class OvertakePhase(Enum):
    NORMAL_DRIVING = 0
    LANE_CHANGE_DEPARTURE = 1
    PASSING_PHASE = 2
    LANE_CHANGE_RETURN = 3

class TrajectoryPlanner(Node):
    def __init__(self):
        super().__init__('pure_pursuit_trajectory_planner')
        
        # Vehicle state
        self.ego_pose = None
        self.ego_twist = None
        self.peer_ego_pose = None
        self.path = []
        self.obstacle_detected = False
        self.vehicle_state = "Idle"
        self.current_overtake_phase = OvertakePhase.NORMAL_DRIVING
        self.is_overtaking = False
        
        # Student goal management
        self.student_list = ["First Student", "Second Student", "Third Student", "Destination Reached"]
        self.student_publish_index = 0
        self.ready_to_publish_student = False
        self.student_published_for_this_path = False
        self.was_at_endpoint = False
        
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
        
        # Vehicle dimensions and safety
        self.ego_length = 0.83
        self.ego_width = 0.38
        self.road_width = 0.87
        
        # Overtaking parameters
        self.overtake_trigger_distance = 1.0
        self.overtake_target_lateral_offset = 0.5
        self.overtake_cruising_length = 3.0
        self.overtake_merge_safety_distance = 2.0
        self.cruising_distance_completed = 0.0
        self.last_ego_position = None
        self.peer_is_behind = False
        
        # DWA parameters
        self.max_linear_vel = self.max_speed
        self.max_angular_vel = math.radians(30.0)
        self.max_linear_accel = 0.5
        self.max_angular_accel = math.radians(60.0)
        self.dt = 0.1
        self.predict_time = 1.5
        self.num_vel_samples = 8
        self.num_yaw_rate_samples = 8
        
        # Control flags
        self.loop_stopped = False
        self.missing_input_warned = False
        
        # Publishers
        self.drive_pub = self.create_publisher(AckermannDrive, '/ackermann_drive', 10)
        self.student_location_pub = self.create_publisher(String, '/student_location', 10)
        self.student_reached_pub = self.create_publisher(PoseStamped, '/student_reached', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/visualization_marker_array', 10)
        
        # Subscribers
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(Path, '/path_data', self.path_callback, 10)
        self.create_subscription(Bool, '/obstacle_detected', self.obstacle_callback, 10)
        self.create_subscription(String, '/vehicle_state', self.state_callback, 10)
        self.create_subscription(Odometry, '/peer_veh_behavior', self.peer_odom_callback, 10)
        
        # Control loop timer (50 Hz)
        self.timer = self.create_timer(0.02, self.control_loop)
        
        self.get_logger().info("Trajectory Planner initialized.")
    
    # --- Utility Functions ---
    def quaternion_to_yaw(self, q):
        """Convert quaternion to yaw angle"""
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)
    
    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle
    
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
        self.peer_ego_pose = msg.pose.pose
    
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
    
    # --- Core Functions ---
    def prune_path(self):
        """Remove path points behind the vehicle"""
        if not self.path or not self.ego_pose:
            return
        
        ego_x, ego_y = self.ego_pose.position.x, self.ego_pose.position.y
        yaw = self.quaternion_to_yaw(self.ego_pose.orientation)
        
        new_path = []
        for wp in self.path:
            dx = wp.pose.position.x - ego_x
            dy = wp.pose.position.y - ego_y
            x_r = math.cos(-yaw) * dx - math.sin(-yaw) * dy  # Forward distance
            
            if x_r > -0.5:  # Keep points not significantly behind
                new_path.append(wp)
        self.path = new_path
    
    def pure_pursuit_steering(self):
        """Calculate steering angle using Pure Pursuit algorithm"""
        if not self.path or not self.ego_pose:
            return 0.0, False
        
        ego_x, ego_y = self.ego_pose.position.x, self.ego_pose.position.y
        yaw = self.quaternion_to_yaw(self.ego_pose.orientation)
        
        # Find lookahead point
        lookahead_point = None
        min_dist_diff = float('inf')
        closest_idx = -1
        
        for i in range(len(self.path) - 1):
            wp1 = self.path[i].pose.position
            wp2 = self.path[i + 1].pose.position
            d1 = math.hypot(wp1.x - ego_x, wp1.y - ego_y)
            d2 = math.hypot(wp2.x - ego_x, wp2.y - ego_y)
            
            # Interpolate if lookahead distance is between waypoints
            if (d1 < self.lookahead_distance <= d2) or (d2 < self.lookahead_distance <= d1):
                ratio = ((self.lookahead_distance - d1) / (d2 - d1)) if (d2 != d1) else 0
                lx = wp1.x + ratio * (wp2.x - wp1.x)
                ly = wp1.y + ratio * (wp2.y - wp1.y)
                lookahead_point = (lx, ly)
                break
            
            # Track closest point as fallback
            dist_diff = abs(self.lookahead_distance - d1)
            if dist_diff < min_dist_diff:
                min_dist_diff = dist_diff
                closest_idx = i
        
        # Use closest point if no interpolation found
        if lookahead_point is None:
            if closest_idx >= 0:
                wp = self.path[closest_idx].pose.position
            else:
                wp = self.path[-1].pose.position
            lookahead_point = (wp.x, wp.y)
        
        # Transform to vehicle frame and calculate curvature
        dx = lookahead_point[0] - ego_x
        dy = lookahead_point[1] - ego_y
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
        
        saturated = False
        if abs(self.smoothed_steering_angle) > self.max_steering_angle_deg:
            saturated = True
            self.smoothed_steering_angle = math.copysign(self.max_steering_angle_deg, self.smoothed_steering_angle)
        
        return self.smoothed_steering_angle, saturated
    
    def generate_dwa_trajectory(self, current_v, current_w):
        """Generate and evaluate DWA trajectories for overtaking"""
        # Dynamic window constraints
        v_min = max(0.1, current_v - self.max_linear_accel * self.dt)
        v_max = min(self.max_linear_vel, current_v + self.max_linear_accel * self.dt)
        w_min = max(-self.max_angular_vel, current_w - self.max_angular_accel * self.dt)
        w_max = min(self.max_angular_vel, current_w + self.max_angular_accel * self.dt)
        
        ego_x, ego_y = self.ego_pose.position.x, self.ego_pose.position.y
        ego_yaw = self.quaternion_to_yaw(self.ego_pose.orientation)
        
        # Generate target pose based on overtaking phase
        if self.current_overtake_phase in [OvertakePhase.LANE_CHANGE_DEPARTURE, OvertakePhase.PASSING_PHASE]:
            # Move laterally away from peer
            peer_x, peer_y = self.peer_ego_pose.position.x, self.peer_ego_pose.position.y
            peer_dx, peer_dy = peer_x - ego_x, peer_y - ego_y
            
            # Avoidance direction (perpendicular to ego-peer vector)
            avoidance_x, avoidance_y = -peer_dy, peer_dx
            avoidance_mag = math.hypot(avoidance_x, avoidance_y)
            if avoidance_mag > 0:
                avoidance_x /= avoidance_mag
                avoidance_y /= avoidance_mag
            
            target_x = ego_x + avoidance_x * self.overtake_target_lateral_offset
            target_y = ego_y + avoidance_y * self.overtake_target_lateral_offset
        else:
            # Return to path
            closest_wp = min(self.path, key=lambda wp: math.hypot(wp.pose.position.x - ego_x, wp.pose.position.y - ego_y))
            target_x, target_y = closest_wp.pose.position.x, closest_wp.pose.position.y
        
        # Find best velocity combination
        best_v, best_w = 0.2, 0.0  # Default safe values
        min_cost = float('inf')
        
        for v in np.linspace(v_min, v_max, self.num_vel_samples):
            for w in np.linspace(w_min, w_max, self.num_yaw_rate_samples):
                # Simulate trajectory
                cost = self.evaluate_trajectory_cost(v, w, target_x, target_y, ego_x, ego_y, ego_yaw)
                
                if cost < min_cost:
                    min_cost = cost
                    best_v, best_w = v, w
        
        return best_v, best_w
    
    def evaluate_trajectory_cost(self, v, w, target_x, target_y, ego_x, ego_y, ego_yaw):
        """Evaluate cost for a trajectory"""
        # Simulate future position
        future_x = ego_x + v * math.cos(ego_yaw) * self.predict_time
        future_y = ego_y + v * math.sin(ego_yaw) * self.predict_time
        
        # Distance to target cost
        target_cost = math.hypot(future_x - target_x, future_y - target_y)
        
        # Peer collision cost
        collision_cost = 0.0
        if self.peer_ego_pose:
            peer_x, peer_y = self.peer_ego_pose.position.x, self.peer_ego_pose.position.y
            peer_dist = math.hypot(future_x - peer_x, future_y - peer_y)
            if peer_dist < 0.5:  # Safety threshold
                collision_cost = 10.0 / (peer_dist + 0.1)
        
        # Speed preference cost
        speed_cost = abs(v - self.max_speed) / self.max_speed
        
        return target_cost + collision_cost + speed_cost
    
    def publish_velocity(self, speed, steering_angle_deg):
        """Publish Ackermann drive command"""
        # Apply steering offset and saturation
        if steering_angle_deg > 0:
            steering_angle_deg += self.steering_offset_deg
        elif steering_angle_deg < 0:
            steering_angle_deg -= self.steering_offset_deg
        
        steering_angle_deg = max(-self.max_steering_angle_deg, min(steering_angle_deg, self.max_steering_angle_deg))
        
        drive_msg = AckermannDrive()
        drive_msg.speed = float(speed)
        drive_msg.steering_angle = float(steering_angle_deg)
        self.drive_pub.publish(drive_msg)
    
    def publish_markers(self):
        """Publish visualization markers for ego and peer vehicles"""
        marker_array = MarkerArray()
        
        # Ego vehicle marker
        if self.ego_pose:
            ego_marker = Marker()
            ego_marker.header.frame_id = "map"
            ego_marker.header.stamp = self.get_clock().now().to_msg()
            ego_marker.ns = "ego_vehicle"
            ego_marker.id = 0
            ego_marker.type = Marker.CUBE
            ego_marker.action = Marker.ADD
            ego_marker.pose.position = self.ego_pose.position
            ego_marker.pose.position.z = 0.1
            ego_marker.pose.orientation = self.ego_pose.orientation
            ego_marker.scale.x, ego_marker.scale.y, ego_marker.scale.z = self.ego_length, self.ego_width, 0.2
            ego_marker.color.a, ego_marker.color.g = 0.5, 1.0
            marker_array.markers.append(ego_marker)
        
        # Peer vehicle marker
        if self.peer_ego_pose:
            peer_marker = Marker()
            peer_marker.header.frame_id = "map"
            peer_marker.header.stamp = self.get_clock().now().to_msg()
            peer_marker.ns = "peer_vehicle"
            peer_marker.id = 1
            peer_marker.type = Marker.CUBE
            peer_marker.action = Marker.ADD
            peer_marker.pose.position = self.peer_ego_pose.position
            peer_marker.pose.position.z = 0.1
            peer_marker.pose.orientation = self.peer_ego_pose.orientation
            peer_marker.scale.x, peer_marker.scale.y, peer_marker.scale.z = self.ego_length, self.ego_width, 0.2
            peer_marker.color.a, peer_marker.color.r = 0.5, 1.0
            marker_array.markers.append(peer_marker)
        
        if marker_array.markers:
            self.marker_pub.publish(marker_array)
    
    def handle_student_goals(self):
        """Manage student pickup/dropoff goals"""
        if not self.path:
            return
        
        goal_pose = self.path[-1].pose
        ego_x, ego_y = self.ego_pose.position.x, self.ego_pose.position.y
        goal_distance = math.hypot(ego_x - goal_pose.position.x, ego_y - goal_pose.position.y)
        at_endpoint = (goal_distance < self.end_threshold)
        
        # Publish student location when newly arriving at endpoint
        if (self.ready_to_publish_student and not self.student_published_for_this_path 
            and self.student_publish_index < len(self.student_list) and at_endpoint and not self.was_at_endpoint):
            
            msg = String()
            msg.data = self.student_list[self.student_publish_index]
            self.student_location_pub.publish(msg)
            print(msg.data)
            
            self.student_publish_index += 1
            self.student_published_for_this_path = True
            self.ready_to_publish_student = False
            
            # Publish reached pose
            reached_pose = PoseStamped()
            reached_pose.header.stamp = self.get_clock().now().to_msg()
            reached_pose.header.frame_id = "map"
            reached_pose.pose = goal_pose
            self.student_reached_pub.publish(reached_pose)
        
        self.was_at_endpoint = at_endpoint
        return at_endpoint
    
    def control_loop(self):
        """Main control loop"""
        self.publish_markers()
        
        if self.loop_stopped:
            return
        
        # Check essential inputs
        missing = []
        if self.ego_pose is None:
            missing.append('ego_pose')
        if self.ego_twist is None:
            missing.append('ego_twist')
        if not self.path:
            missing.append('path_data')
        
        if missing:
            if not self.missing_input_warned:
                self.get_logger().warn(f"Missing inputs: {', '.join(missing)}")
                self.missing_input_warned = True
            return
        else:
            self.missing_input_warned = False
        
        # Prune path and handle goal management
        self.prune_path()
        if not self.path:
            self.publish_velocity(0.0, 0.0)
            self.loop_stopped = True
            return
        
        at_endpoint = self.handle_student_goals()
        
        # Stopping conditions
        if (at_endpoint or self.obstacle_detected or 
            self.vehicle_state.strip() not in ["Driving"]):
            self.publish_velocity(0.0, 0.0)
            if at_endpoint:
                self.loop_stopped = True
            self.current_overtake_phase = OvertakePhase.NORMAL_DRIVING
            self.is_overtaking = False
            return
        
        # Calculate peer distance
        peer_distance = float('inf')
        if self.peer_ego_pose and self.ego_pose:
            peer_distance = math.hypot(
                self.peer_ego_pose.position.x - self.ego_pose.position.x,
                self.peer_ego_pose.position.y - self.ego_pose.position.y
            )
        
        # Control decision: DWA for overtaking, Pure Pursuit otherwise
        if self.peer_ego_pose and peer_distance <= self.overtake_trigger_distance:
            # Overtaking logic
            if not self.is_overtaking:
                self.is_overtaking = True
                self.current_overtake_phase = OvertakePhase.LANE_CHANGE_DEPARTURE
                self.cruising_distance_completed = 0.0
                self.get_logger().info("Starting overtake maneuver")
            
            # Update overtaking phase based on peer position
            ego_yaw = self.quaternion_to_yaw(self.ego_pose.orientation)
            ego_x, ego_y = self.ego_pose.position.x, self.ego_pose.position.y
            peer_x, peer_y = self.peer_ego_pose.position.x, self.peer_ego_pose.position.y
            
            # Check if peer is behind
            dx_peer = peer_x - ego_x
            dy_peer = peer_y - ego_y
            peer_local_x = math.cos(-ego_yaw) * dx_peer - math.sin(-ego_yaw) * dy_peer
            self.peer_is_behind = (peer_local_x < -0.5)
            
            # Phase management
            if self.peer_is_behind and self.cruising_distance_completed >= self.overtake_cruising_length:
                if peer_distance >= self.overtake_merge_safety_distance:
                    self.current_overtake_phase = OvertakePhase.LANE_CHANGE_RETURN
                else:
                    self.current_overtake_phase = OvertakePhase.PASSING_PHASE
            elif not self.peer_is_behind:
                self.current_overtake_phase = OvertakePhase.LANE_CHANGE_DEPARTURE
            else:
                self.current_overtake_phase = OvertakePhase.PASSING_PHASE
            
            # Update cruising distance
            if self.last_ego_position and self.peer_is_behind:
                distance_increment = math.hypot(ego_x - self.last_ego_position[0], ego_y - self.last_ego_position[1])
                self.cruising_distance_completed += distance_increment
            
            self.last_ego_position = (ego_x, ego_y)
            
            # Use DWA for trajectory planning
            current_v = self.ego_twist.linear.x
            current_w = self.ego_twist.angular.z
            target_v, target_w = self.generate_dwa_trajectory(current_v, current_w)
            
            # Convert to steering angle
            if abs(target_v) > 0.05:
                steering_angle_deg = math.degrees(math.atan(self.wheelbase * target_w / target_v))
            else:
                steering_angle_deg = 0.0
            
            self.get_logger().info(f"DWA Overtaking: Phase={self.current_overtake_phase.name}, "
                                 f"V={target_v:.2f}, Steer={steering_angle_deg:.2f}°")
            
        else:
            # Normal Pure Pursuit driving
            if self.is_overtaking:
                self.is_overtaking = False
                self.current_overtake_phase = OvertakePhase.NORMAL_DRIVING
                self.get_logger().info("Overtaking complete - returning to Pure Pursuit")
            
            steering_angle_deg, saturated = self.pure_pursuit_steering()
            
            # Speed based on steering angle
            abs_steer = min(abs(steering_angle_deg), self.max_steering_angle_deg)
            speed_range = self.max_speed - self.min_speed
            target_v = self.max_speed - (abs_steer / self.max_steering_angle_deg) * speed_range
            target_v = max(self.min_speed, min(target_v, self.max_speed))
        
        # Publish control command (negative steering for vehicle convention)
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
        rclpy.shutdown()

if __name__ == '__main__':
    main()