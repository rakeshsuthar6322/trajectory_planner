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
        
        # Overtaking parameters
        self.overtake_trigger_distance = 1.50
        self.overtake_target_lateral_offset = 0.35
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
    
    def generate_dwa_trajectory(self, current_v, current_w):
        """Generate and evaluate DWA trajectories for overtaking"""
        # Dynamic window constraints
        v_min = max(0.1, current_v - self.max_linear_accel * self.dt)
        v_max = min(self.max_linear_vel, current_v + self.max_linear_accel * self.dt)
        w_min = max(-self.max_angular_vel, current_w - self.max_angular_accel * self.dt)
        w_max = min(self.max_angular_vel, current_w + self.max_angular_accel * self.dt)
        
        ego_pos = self.ego_pose.position
        ego_yaw = self.quaternion_to_yaw(self.ego_pose.orientation)
        
        # Get target based on overtaking phase
        target_x, target_y = self.get_overtaking_target(ego_pos, ego_yaw)
        
        # Find best velocity combination
        best_v, best_w = 0.2, 0.0
        min_cost = float('inf')
        
        for v in np.linspace(v_min, v_max, self.num_vel_samples):
            for w in np.linspace(w_min, w_max, self.num_yaw_rate_samples):
                cost = self.evaluate_trajectory_cost(v, w, target_x, target_y, ego_pos.x, ego_pos.y, ego_yaw)
                if cost < min_cost:
                    min_cost = cost
                    best_v, best_w = v, w
        
        return best_v, best_w
    
    def get_overtaking_target(self, ego_pos, ego_yaw):
        """Get target position for overtaking based on current phase"""
        if self.current_overtake_phase in [OvertakePhase.LANE_CHANGE_DEPARTURE, OvertakePhase.PASSING_PHASE]:
            # Find path target
            path_target = self.find_path_target_ahead(ego_pos, ego_yaw)
            
            # Calculate avoidance direction
            peer_pos = self.peer_ego_pose.position
            peer_dx, peer_dy = peer_pos.x - ego_pos.x, peer_pos.y - ego_pos.y
            
            avoidance_x, avoidance_y = -peer_dy, peer_dx
            avoidance_mag = math.hypot(avoidance_x, avoidance_y)
            if avoidance_mag > 0:
                avoidance_x /= avoidance_mag
                avoidance_y /= avoidance_mag
            
            if self.current_overtake_phase == OvertakePhase.PASSING_PHASE:
                path_weight, avoidance_weight = 0.3, 0.7
            else:
                path_weight, avoidance_weight = 0.5, 0.5
            
            target_x = (path_weight * path_target[0] + 
                       avoidance_weight * (ego_pos.x + avoidance_x * self.overtake_target_lateral_offset))
            target_y = (path_weight * path_target[1] + 
                       avoidance_weight * (ego_pos.y + avoidance_y * self.overtake_target_lateral_offset))
        else:
            # LANE_CHANGE_RETURN: Return to path
            target_x, target_y = self.find_closest_path_point_ahead(ego_pos, ego_yaw)
            self.get_logger().info("MERGING BACK TO PATH")
        
        return target_x, target_y
    
    def find_path_target_ahead(self, ego_pos, ego_yaw):
        """Find path target ahead of vehicle"""
        min_ahead_dist = float('inf')
        target_x, target_y = self.path[-1].pose.position.x, self.path[-1].pose.position.y
        
        for wp in self.path:
            wp_pos = wp.pose.position
            dx, dy = wp_pos.x - ego_pos.x, wp_pos.y - ego_pos.y
            x_local = math.cos(-ego_yaw) * dx - math.sin(-ego_yaw) * dy
            
            if x_local > 0:
                dist = math.hypot(dx, dy)
                if self.lookahead_distance <= dist <= self.lookahead_distance * 2 and dist < min_ahead_dist:
                    min_ahead_dist = dist
                    target_x, target_y = wp_pos.x, wp_pos.y
        
        return target_x, target_y
    
    def find_closest_path_point_ahead(self, ego_pos, ego_yaw):
        """Find closest path point ahead for merging"""
        min_path_dist = float('inf')
        target_x, target_y = None, None
        
        for wp in self.path:
            wp_pos = wp.pose.position
            dx, dy = wp_pos.x - ego_pos.x, wp_pos.y - ego_pos.y
            x_local = math.cos(-ego_yaw) * dx - math.sin(-ego_yaw) * dy
            
            if x_local > 0:
                dist = math.hypot(dx, dy)
                if dist < min_path_dist:
                    min_path_dist = dist
                    target_x, target_y = wp_pos.x, wp_pos.y
        
        if target_x is None:
            closest_wp = min(self.path, key=lambda wp: self.get_distance(wp.pose.position, ego_pos))
            target_x, target_y = closest_wp.pose.position.x, closest_wp.pose.position.y
        
        return target_x, target_y
    
    def evaluate_trajectory_cost(self, v, w, target_x, target_y, ego_x, ego_y, ego_yaw):
        """Evaluate cost for a trajectory"""
        trajectory = self.simulate_trajectory(v, w, ego_x, ego_y, ego_yaw)
        if not trajectory:
            return float('inf')
        
        final_x, final_y, final_yaw = trajectory[-1]
        
        target_cost = math.hypot(final_x - target_x, final_y - target_y)
        clearance_cost = self.calculate_clearance_cost(trajectory)
        speed_cost = abs(v - self.max_speed * 0.9) / self.max_speed
        path_cost = self.calculate_path_cost(final_x, final_y)
        
        forward_bonus = (v / self.max_speed) * 0.5 if v >= 0.15 else -2.0
        
        if self.current_overtake_phase == OvertakePhase.LANE_CHANGE_RETURN:
            weights = [0.2, 0.2, 0.1, 0.5]
        else:
            weights = [0.4, 0.3, 0.1, 0.2]
        
        costs = [target_cost, clearance_cost, speed_cost, path_cost]
        return sum(w * c for w, c in zip(weights, costs)) - forward_bonus
    
    def simulate_trajectory(self, v, w, ego_x, ego_y, ego_yaw):
        """Simulate trajectory for given velocities"""
        trajectory = []
        x, y, yaw = ego_x, ego_y, ego_yaw
        
        for t in np.arange(0, self.predict_time, self.dt):
            if abs(w) < 1e-3:
                x += v * math.cos(yaw) * self.dt
                y += v * math.sin(yaw) * self.dt
            else:
                x += v/w * (math.sin(yaw + w*self.dt) - math.sin(yaw))
                y += v/w * (-math.cos(yaw + w*self.dt) + math.cos(yaw))
                yaw = self.normalize_angle(yaw + w * self.dt)
            trajectory.append((x, y, yaw))
        
        return trajectory
    
    def calculate_clearance_cost(self, trajectory):
        """Calculate collision avoidance cost"""
        if not self.peer_ego_pose:
            return 0.0
        
        peer_pos = self.peer_ego_pose.position
        min_clearance = min(math.hypot(x - peer_pos.x, y - peer_pos.y) for x, y, _ in trajectory)
        
        safety_radius = (self.ego_width / 2) + 0.2
        
        if min_clearance < safety_radius:
            return 5.0
        else:
            return max(0, (0.5 - min_clearance) * 1.0)
    
    def calculate_path_cost(self, final_x, final_y):
        """Calculate path deviation cost"""
        if not self.path:
            return 0.0
        
        min_dist = min(math.hypot(final_x - wp.pose.position.x, final_y - wp.pose.position.y) 
                      for wp in self.path)
        
        return min_dist * (1.5 if self.current_overtake_phase == OvertakePhase.LANE_CHANGE_RETURN else 0.3)
    
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
    
    def publish_markers(self):
        """Publish visualization markers"""
        marker_array = MarkerArray()
        
        def create_marker(pose, ns, marker_id, color):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = ns
            marker.id = marker_id
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position = pose.position
            marker.pose.position.z = 0.1
            marker.pose.orientation = pose.orientation
            marker.scale.x, marker.scale.y, marker.scale.z = self.ego_length, self.ego_width, 0.2
            marker.color.a = 0.5
            if color == 'green':
                marker.color.g = 1.0
            else:
                marker.color.r = 1.0
            return marker
        
        if self.ego_pose:
            marker_array.markers.append(create_marker(self.ego_pose, "ego_vehicle", 0, 'green'))
        
        if self.peer_ego_pose:
            marker_array.markers.append(create_marker(self.peer_ego_pose, "peer_vehicle", 1, 'red'))
        
        if marker_array.markers:
            self.marker_pub.publish(marker_array)
    
    def handle_student_goals(self):
        """Manage student pickup/dropoff goals"""
        if not self.path:
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
    
    def update_overtaking_phase(self, peer_distance):
        """Update overtaking phase based on peer position and distance"""
        ego_pos = self.ego_pose.position
        ego_yaw = self.quaternion_to_yaw(self.ego_pose.orientation)
        peer_pos = self.peer_ego_pose.position
        
        dx_peer = peer_pos.x - ego_pos.x
        dy_peer = peer_pos.y - ego_pos.y
        peer_local_x = math.cos(-ego_yaw) * dx_peer - math.sin(-ego_yaw) * dy_peer
        self.peer_is_behind = (peer_local_x < -0.5)
        
        if (self.peer_is_behind and self.cruising_distance_completed >= self.overtake_cruising_length 
            and peer_distance >= self.overtake_merge_safety_distance):
            self.current_overtake_phase = OvertakePhase.LANE_CHANGE_RETURN
        elif not self.peer_is_behind:
            self.current_overtake_phase = OvertakePhase.LANE_CHANGE_DEPARTURE
        else:
            self.current_overtake_phase = OvertakePhase.PASSING_PHASE
        
        min_dist_to_path = min(self.get_distance(ego_pos, wp.pose.position) for wp in self.path)
        if min_dist_to_path > 0.8 and self.current_overtake_phase != OvertakePhase.LANE_CHANGE_RETURN:
            self.current_overtake_phase = OvertakePhase.LANE_CHANGE_RETURN
            self.get_logger().info("Forcing merge back - too far from path")
        
        if self.last_ego_position and self.peer_is_behind:
            distance_increment = math.hypot(ego_pos.x - self.last_ego_position[0], 
                                          ego_pos.y - self.last_ego_position[1])
            self.cruising_distance_completed += distance_increment
        
        self.last_ego_position = (ego_pos.x, ego_pos.y)
    
    def control_loop(self):
        """Main control loop"""
        self.publish_markers()
        
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
            self.current_overtake_phase = OvertakePhase.NORMAL_DRIVING
            self.is_overtaking = False
            return
        
        peer_distance = float('inf')
        if self.peer_ego_pose:
            peer_distance = self.get_distance(self.ego_pose.position, self.peer_ego_pose.position)
        
        if self.peer_ego_pose and peer_distance <= self.overtake_trigger_distance:
            if not self.is_overtaking:
                self.is_overtaking = True
                self.current_overtake_phase = OvertakePhase.LANE_CHANGE_DEPARTURE
                self.cruising_distance_completed = 0.0
                self.get_logger().info("Starting overtake maneuver")
            
            self.update_overtaking_phase(peer_distance)
            
            current_v = self.ego_twist.linear.x
            current_w = self.ego_twist.angular.z
            target_v, target_w = self.generate_dwa_trajectory(current_v, current_w)
            
            steering_angle_deg = (math.degrees(math.atan(self.wheelbase * target_w / target_v)) 
                                if abs(target_v) > 0.05 else 0.0)
            
            self.get_logger().info(f"DWA Overtaking: Phase={self.current_overtake_phase.name}, "
                                 f"V={target_v:.2f}, Steer={steering_angle_deg:.2f}°")
        else:
            if self.is_overtaking:
                self.is_overtaking = False
                self.current_overtake_phase = OvertakePhase.NORMAL_DRIVING
                self.get_logger().info("Overtaking complete - returning to Pure Pursuit")
            
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