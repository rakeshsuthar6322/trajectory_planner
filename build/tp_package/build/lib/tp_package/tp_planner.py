"""Trajectory planner node for autonomous vehicle path following using pure pursuit algorithm.

Publishes steering and speed commands to AckermannDrive message based on odometry and path input.
Also implements student logic for multi-goal publishing.
"""

import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import Bool, String
from ackermann_msgs.msg import AckermannDrive
from geometry_msgs.msg import PoseStamped

def quaternion_to_yaw(q):
    """Convert quaternion to yaw angle."""
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

class TrajectoryPlanner(Node):
    """Trajectory planner node for path following and student location publishing."""

    def __init__(self):
        """Initialize the trajectory planner node."""
        super().__init__('trajectory_planner')
        self.ego_pose = None
        self.ego_twist = None
        self.path = []
        self.obstacle_detected = False
        self.vehicle_state = "Idle"

        self.drive_pub = self.create_publisher(AckermannDrive, '/ackermann_drive', 10)
        self.student_location_pub = self.create_publisher(String, '/student_location', 10)
        self.student_reached_pub = self.create_publisher(PoseStamped, '/student_reached', 10)

        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(Path, '/path_data', self.path_callback, 10)
        self.create_subscription(Bool, '/obstacle_detected', self.obstacle_callback, 10)
        self.create_subscription(String, '/vehicle_state', self.state_callback, 10)
        self.create_subscription(AckermannDrive, '/ackermann_drive_feedback', self.feedback_callback, 10)

        self.timer = self.create_timer(0.02, self.control_loop)

        # --- student logic ---
        self.student_list = [
            "First Student", "Second Student", "Third Student", "Destination Reached"
        ]
        self.student_publish_index = 0  # index in student_list
        self.ready_to_publish_student = False  # set True in path_callback
        self.student_published_for_this_path = False  # published flag, reset with new path
        self.was_at_endpoint = False  # Track if robot was at endpoint in previous loop

        self.loop_stopped = False
        self.missing_input_warned = False

        self.min_speed = 0.3
        self.max_speed = 0.4
        self.max_steering_angle_deg = 30.0
        self.lookahead_distance = 0.7
        self.wheelbase = 0.50
        self.steering_smoothing_factor = 0.2
        self.smoothed_steering_angle = 0.0
        self.steering_offset_deg = 4.0

    def odom_callback(self, msg):
        """Odometry callback to update ego pose and twist."""
        self.ego_pose = msg.pose.pose
        self.ego_twist = msg.twist.twist
        wheelbase_half = self.wheelbase / 2.0
        q = self.ego_pose.orientation
        yaw = quaternion_to_yaw(q)
        self.ego_pose.position.x += wheelbase_half * math.cos(yaw)
        self.ego_pose.position.y += wheelbase_half * math.sin(yaw)

    def path_callback(self, msg):
        """Path callback to update the planned path."""
        self.path = msg.poses
        self.loop_stopped = False
        # Core reset for student publishing logic:
        self.student_published_for_this_path = False
        self.ready_to_publish_student = True  # allow publishing when endpoint reached

    def obstacle_callback(self, msg):
        """Obstacle callback to update obstacle detection state."""
        self.obstacle_detected = msg.data

    def state_callback(self, msg):
        """Vehicle state callback to update the current vehicle state."""
        self.vehicle_state = msg.data

    def feedback_callback(self, msg):
        """Ackermann drive feedback callback (currently unused)."""
        pass

    def prune_path(self):
        """Prune waypoints behind the vehicle from the path."""
        if not self.path or not self.ego_pose:
            return
        ego_x = self.ego_pose.position.x
        ego_y = self.ego_pose.position.y
        q = self.ego_pose.orientation
        yaw = quaternion_to_yaw(q)
        new_path = []
        for wp in self.path:
            dx = wp.pose.position.x - ego_x
            dy = wp.pose.position.y - ego_y
            x_r = math.cos(-yaw) * dx - math.sin(-yaw) * dy
            if x_r > -0.5:
                new_path.append(wp)
        self.path = new_path

    def control_loop(self):
        """Main control loop for trajectory planning and student publishing."""
        if self.loop_stopped:
            return

        missing = []
        if self.ego_pose is None:
            missing.append('ego_pose')
        if self.ego_twist is None:
            missing.append('ego_twist')
        if not self.path:
            missing.append('path_data')

        if missing:
            if not self.missing_input_warned:
                self.get_logger().warn(
                    f"check your inputs buddy (s): {', '.join(missing)}"
                )
                self.missing_input_warned = True
            return

        self.missing_input_warned = False
        self.prune_path()

        if not self.path:
            self.get_logger().warn(
                "All path points are behind the vehicle. Stopping and requesting new path."
            )
            self.publish_velocity(0.0, 0.0)
            self.loop_stopped = True
            return

        goal_pose = self.path[-1].pose
        ego_x = self.ego_pose.position.x
        ego_y = self.ego_pose.position.y
        goal_x = goal_pose.position.x
        goal_y = goal_pose.position.y
        goal_distance = math.hypot(ego_x - goal_x, ego_y - goal_y)
        end_threshold = 0.5  # meters

        at_endpoint = goal_distance < end_threshold

        # Only publish when newly arriving at the endpoint (not already there)
        if (self.ready_to_publish_student
                and not self.student_published_for_this_path
                and self.student_publish_index < len(self.student_list)
                and at_endpoint
                and not self.was_at_endpoint):
            msg = String()
            msg.data = self.student_list[self.student_publish_index]
            self.student_location_pub.publish(msg)
            self.get_logger().info(msg.data)  # print in terminal as per your requirement

            self.student_publish_index += 1
            self.student_published_for_this_path = True
            self.ready_to_publish_student = False

            # Optionally, publish the reached pose for multi-goal planner
            reached_pose = PoseStamped()
            reached_pose.header.stamp = self.get_clock().now().to_msg()
            reached_pose.header.frame_id = "map"
            reached_pose.pose = goal_pose
            self.student_reached_pub.publish(reached_pose)

        # Update the endpoint status for the next control loop
        self.was_at_endpoint = at_endpoint

        # --- normal stopping logic after reaching endpoint ---
        if goal_distance < end_threshold:
            self.publish_velocity(0.0, 0.0)
            self.loop_stopped = True
            return

        if self.obstacle_detected:
            self.publish_velocity(0.0, 0.0)
            return

        if self.vehicle_state.strip() == "Boarding":
            self.publish_velocity(0.0, 0.0)
            return

        if self.vehicle_state.strip() != "Driving":
            self.publish_velocity(0.0, 0.0)
            return

        # --- Steering and speed logic ---
        steering_angle_deg, _ = self.pure_pursuit_steering()
        abs_steer = min(abs(steering_angle_deg), self.max_steering_angle_deg)
        speed_range = self.max_speed - self.min_speed
        target_speed = self.max_speed - (abs_steer / self.max_steering_angle_deg) * speed_range
        target_speed = max(self.min_speed, min(target_speed, self.max_speed))

        self.publish_velocity(target_speed, -steering_angle_deg)

    def pure_pursuit_steering(self):
        """Compute pure pursuit steering angle based on the path and ego pose."""
        if not self.path or not self.ego_pose:
            return 0.0, False

        ego_x = self.ego_pose.position.x
        ego_y = self.ego_pose.position.y
        q = self.ego_pose.orientation
        yaw = quaternion_to_yaw(q)

        lookahead_point = None
        min_dist_diff = float('inf')
        closest_idx = -1
        for i in range(len(self.path) - 1):
            wp1 = self.path[i].pose.position
            wp2 = self.path[i + 1].pose.position
            dx1, dy1 = wp1.x - ego_x, wp1.y - ego_y
            dx2, dy2 = wp2.x - ego_x, wp2.y - ego_y
            d1 = math.hypot(dx1, dy1)
            d2 = math.hypot(dx2, dy2)
            if (d1 < self.lookahead_distance <= d2) or (d2 < self.lookahead_distance <= d1):
                ratio = ((self.lookahead_distance - d1) / (d2 - d1)) if (d2 != d1) else 0
                lx = wp1.x + ratio * (wp2.x - wp1.x)
                ly = wp1.y + ratio * (wp2.y - wp1.y)
                lookahead_point = (lx, ly)
                break
            dist_diff = abs(self.lookahead_distance - d1)
            if dist_diff < min_dist_diff:
                min_dist_diff = dist_diff
                closest_idx = i

        if lookahead_point is None:
            if closest_idx >= 0:
                wp = self.path[closest_idx].pose.position
                lookahead_point = (wp.x, wp.y)
            else:
                wp = self.path[-1].pose.position
                lookahead_point = (wp.x, wp.y)

        dx = lookahead_point[0] - ego_x
        dy = lookahead_point[1] - ego_y
        x_r = math.cos(-yaw) * dx - math.sin(-yaw) * dy
        y_r = math.sin(-yaw) * dx + math.cos(-yaw) * dy

        if x_r <= 0:
            self.get_logger().warn(
                "Lookahead point is behind the vehicle. Stopping and requesting new path."
            )
            return 0.0, False

        wheelbase = self.wheelbase
        ld = math.hypot(x_r, y_r)
        if ld < 1e-3:
            return 0.0, False
        curvature = (2 * y_r) / (ld * ld)
        steering_angle_rad = math.atan(wheelbase * curvature)
        steering_angle_deg = math.degrees(steering_angle_rad)

        self.smoothed_steering_angle = (
            (1 - self.steering_smoothing_factor) * self.smoothed_steering_angle
            + self.steering_smoothing_factor * steering_angle_deg
        )

        saturated = False
        if self.smoothed_steering_angle > self.max_steering_angle_deg:
            saturated = True
            self.smoothed_steering_angle = self.max_steering_angle_deg
        elif self.smoothed_steering_angle < -self.max_steering_angle_deg:
            saturated = True
            self.smoothed_steering_angle = -self.max_steering_angle_deg

        return self.smoothed_steering_angle, saturated

    def publish_velocity(self, speed, steering_angle_deg=0.0):
        """Publish Ackermann drive command with limited and offset steering angle."""
        if steering_angle_deg > 0:
            steering_angle_deg += self.steering_offset_deg
        elif steering_angle_deg < 0:
            steering_angle_deg -= self.steering_offset_deg
        steering_angle_deg = max(
            -self.max_steering_angle_deg, min(steering_angle_deg, self.max_steering_angle_deg)
        )
        drive_msg = AckermannDrive()
        drive_msg.speed = float(speed)
        drive_msg.steering_angle = float(steering_angle_deg)
        drive_msg.steering_angle_velocity = 0.0
        drive_msg.acceleration = 0.0
        drive_msg.jerk = 0.0
        self.drive_pub.publish(drive_msg)

        if not self.loop_stopped:
            self.get_logger().info(
                "[DRIVE] speed: %.2f m/s | steering_angle: %.2f deg", speed, steering_angle_deg
            )

def main(args=None):
    """Main entry point for the trajectory planner node."""
    rclpy.init(args=args)
    node = TrajectoryPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()