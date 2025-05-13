import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Path
from std_msgs.msg import Bool, String
from ackermann_msgs.msg import AckermannDrive
import math
import tf_transformations


class TrajectoryPlanner(Node):
    def __init__(self):
        super().__init__('trajectory_planner')

        # Internal state
        self.ego_pose = None
        self.ego_twist = None
        self.path = []
        self.obstacle_detected = False
        self.vehicle_state = "Idle"
        self.reached_end = False
        self.published_student_location = False

        self.feedback_speed = 0.0
        self.feedback_steering_angle = 0.0

        self.min_speed = 1.0
        self.max_speed = 3.0

        self.start_time = self.get_clock().now()

        # Publishers
        self.drive_pub = self.create_publisher(AckermannDrive, '/ackermann_drive', 10)
        self.student_location_pub = self.create_publisher(String, '/student_location', 10)

        # Subscribers
        self.create_subscription(PoseStamped, '/ego_pose', self.pose_callback, 10)
        self.create_subscription(TwistStamped, '/ego_twist', self.twist_callback, 10)
        self.create_subscription(Path, '/path_data', self.path_callback, 10)
        self.create_subscription(Bool, '/obstacle_detected', self.obstacle_callback, 10)
        self.create_subscription(String, '/vehicle_state', self.state_callback, 10)
        self.create_subscription(AckermannDrive, '/ackermann_drive_feedback', self.feedback_callback, 10)

        self.create_timer(0.1, self.control_loop)

        # Load hardcoded path initially
        self.load_hardcoded_path()

    def load_hardcoded_path(self):
        waypoints = [
            (3, 6.5), (3.5, 6.5), (3.5, 5.8), (3.5, 5.0), (3.5, 4.8),
            (3.5, 4.5), (3.5, 4.2), (3.5, 4.0), (3.5, 3.9), (3.5, 3.7),
            (3.5, 3.5), (3.5, 3.2), (3.5, 3.0), (3.5, 2.8), (3.5, 2.5),
            (3.5, 2.3), (3.5, 1.0), (3.0, 0.5), (2.5, 0.5), (2.2, 0.5),
            (2.0, 0.5), (1.8, 0.5), (1.5, 0.5), (1.4, 0.5), (1.2, 0.5),
            (1.0, 0.5), (0.5, 1.0), (0.5, 1.2), (0.5, 1.5), (0.5, 2.0),
            (0.5, 2.2), (0.5, 3.0), (0.5, 3.4), (0.5, 4.0), (0.5, 4.2),
            (0.5, 4.7), (0.5, 5.0), (0.5, 5.5), (0.5, 5.8), (0.5, 6.0),
            (0.5, 6.4), (0.5, 6.8), (0.5, 7.0)
        ]
        for x, y in waypoints:
            pose = PoseStamped()
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            self.path.append(pose)
        self.get_logger().info(f"Loaded {len(self.path)} hardcoded waypoints.")

    def pose_callback(self, msg):
        self.ego_pose = msg.pose

    def twist_callback(self, msg):
        self.ego_twist = msg.twist

    def path_callback(self, msg):
        self.path = msg.poses
        self.reached_end = False
        self.published_student_location = False
        self.start_time = self.get_clock().now()
        self.get_logger().info(f"Received new path with {len(self.path)} waypoints.")

    def obstacle_callback(self, msg):
        self.obstacle_detected = msg.data
        status = "true" if self.obstacle_detected else "false"
        self.get_logger().info(f"Obstacle detected: {status}")

    def state_callback(self, msg):
        self.vehicle_state = msg.data
        self.get_logger().info(f"Vehicle state: {self.vehicle_state}")

    def feedback_callback(self, msg):
        self.feedback_speed = msg.speed
        self.feedback_steering_angle = msg.steering_angle

    def control_loop(self):
        if not self.ego_pose or not self.ego_twist:
            self.get_logger().warn("Missing ego pose or twist data.")
            return

        if self.path:
            goal_position = self.path[-1].pose.position
            distance_to_goal = self.euclidean_distance(self.ego_pose.position, goal_position)
            if distance_to_goal <= 0.5:
                self.reached_end = True

        if self.reached_end:
            if not self.published_student_location:
                student_msg = String()
                student_msg.data = "First Student"
                self.student_location_pub.publish(student_msg)
                self.published_student_location = True
                self.get_logger().info("Published student_location: First Student")
            self.publish_velocity(0.0, 0.0)
            return

        if self.obstacle_detected:
            self.get_logger().info("Stopping: Obstacle detected.")
            self.publish_velocity(0.0, 0.0)
            return

        if self.vehicle_state.strip().lower() != "driving":
            self.get_logger().info(f"Stopping: Vehicle state is '{self.vehicle_state}'.")
            self.publish_velocity(0.0, 0.0)
            return

        current_time = self.get_clock().now()
        elapsed_time = (current_time - self.start_time).nanoseconds * 1e-9
        ramp_up_time = 5.0
        final_speed = self.max_speed
        base_speed = (elapsed_time / ramp_up_time) * final_speed if elapsed_time < ramp_up_time else final_speed

        lookahead_distance = 1.5 + (base_speed / self.max_speed) * 2.0
        target_point = None
        for pose_stamped in self.path:
            dist = self.euclidean_distance(self.ego_pose.position, pose_stamped.pose.position)
            if dist >= lookahead_distance:
                target_point = pose_stamped.pose.position
                break
        if not target_point and self.path:
            target_point = self.path[-1].pose.position

        yaw = self.get_yaw_from_pose(self.ego_pose)
        dx = target_point.x - self.ego_pose.position.x
        dy = target_point.y - self.ego_pose.position.y
        angle_to_target = math.atan2(dy, dx)
        steering_angle = math.atan2(math.sin(angle_to_target - yaw), math.cos(angle_to_target - yaw))

        max_steering_angle = math.radians(30)
        steering_angle = max(-max_steering_angle, min(steering_angle, max_steering_angle))

        angle_deg = abs(math.degrees(steering_angle))
        if angle_deg > 20:
            target_speed = self.min_speed
        elif angle_deg > 10:
            target_speed = (self.min_speed + self.max_speed) / 2
        else:
            target_speed = base_speed

        speed_error = abs(self.feedback_speed - target_speed)
        steer_error = abs(self.feedback_steering_angle - steering_angle)
        self.get_logger().info(
            f"Target: V={target_speed:.2f}, Steer={math.degrees(steering_angle):.2f}° | "
            f"Feedback: V={self.feedback_speed:.2f}, Steer={self.feedback_steering_angle:.2f}"
        )

        self.publish_velocity(target_speed, steering_angle)

    def publish_velocity(self, speed, steering_angle=0.0):
        drive_msg = AckermannDrive()
        drive_msg.speed = float(speed)
        drive_msg.steering_angle = float(steering_angle)
        drive_msg.steering_angle_velocity = 0.0
        drive_msg.acceleration = 0.0
        drive_msg.jerk = 0.0

        self.drive_pub.publish(drive_msg)

        self.get_logger().info(
            f"Publishing AckermannDrive → "
            f"Speed: {drive_msg.speed:.2f}, "
            f"Steering Angle: {drive_msg.steering_angle:.2f}"
        )

    @staticmethod
    def euclidean_distance(p1, p2):
        return math.sqrt((p1.x - p2.x) ** 2 + (p1.y - p2.y) ** 2)

    def get_yaw_from_pose(self, pose):
        q = pose.orientation
        _, _, yaw = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        return yaw


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
