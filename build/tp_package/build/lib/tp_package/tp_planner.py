import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Path
from std_msgs.msg import Bool, String
from ackermann_msgs.msg import AckermannDrive
import math


class TrajectoryPlanner(Node):
    def __init__(self):
        super().__init__('trajectory_planner')

        # Internal state
        self.ego_pose = None
        self.ego_twist = None
        self.path = []
        self.obstacle_detected = False
        self.vehicle_state = "Idle"  # "Idle" or "Driving"
        self.reached_end = False

        # Feedback state
        self.feedback_speed = 0.0
        self.feedback_steering_angle = 0.0

        # Velocity constraints
        self.min_speed = 1.0
        self.max_speed = 2.0

        # Time tracking
        self.start_time = self.get_clock().now()

        # Publishers
        self.drive_pub = self.create_publisher(AckermannDrive, '/ackermann_drive', 10)

        # Subscribers
        self.create_subscription(PoseStamped, '/ego_pose', self.pose_callback, 10)
        self.create_subscription(TwistStamped, '/ego_twist', self.twist_callback, 10)
        self.create_subscription(Path, '/path_data', self.path_callback, 10)
        self.create_subscription(Bool, '/obstacle_detected', self.obstacle_callback, 10)
        self.create_subscription(String, '/vehicle_state', self.state_callback, 10)
        self.create_subscription(AckermannDrive, '/ackermann_drive_feedback', self.feedback_callback, 10)

        # Timer for periodic processing
        self.create_timer(0.1, self.control_loop)  # 10 Hz

    def pose_callback(self, msg):
        self.ego_pose = msg.pose

    def twist_callback(self, msg):
        self.ego_twist = msg.twist

    def path_callback(self, msg):
        self.path = msg.poses
        self.reached_end = False
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
        # Check required data
        if not self.ego_pose or not self.ego_twist:
            return

        # Normalize vehicle state string
        state = self.vehicle_state.strip().lower()

        # Stop the car in any blocking condition
        if self.obstacle_detected:
            self.get_logger().info("Stopping: Obstacle detected.")
            self.publish_velocity(0.0, 0.0)
            return

        if state != "driving":
            self.get_logger().info(f"Stopping: Vehicle state is '{self.vehicle_state}'.")
            self.publish_velocity(0.0, 0.0)
            return

        if self.reached_end:
            self.get_logger().info("Stopping: Reached end of path.")
            self.publish_velocity(0.0, 0.0)
            return

        # Get time-based speed
        current_time = self.get_clock().now()
        elapsed_time = (current_time - self.start_time).nanoseconds * 1e-9
        ramp_up_time = 5.0
        final_speed = self.max_speed

        if elapsed_time < ramp_up_time:
            target_speed = (elapsed_time / ramp_up_time) * final_speed
        else:
            target_speed = final_speed

        target_steering_angle = 0.0

        # Feedback log
        speed_error = abs(self.feedback_speed - target_speed)
        steer_error = abs(self.feedback_steering_angle - target_steering_angle)

        self.get_logger().info(
            f"Target: V={target_speed:.2f}, Steer={target_steering_angle:.2f} | "
            f"Feedback: V={self.feedback_speed:.2f}, Steer={self.feedback_steering_angle:.2f} | "
            f"Error: V={speed_error:.2f}, Steer={steer_error:.2f}"
        )

        self.publish_velocity(target_speed, target_steering_angle)

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

    def get_next_waypoint(self):
        if not self.path:
            return None

        closest_index = None
        min_dist = float('inf')

        for i, pose_stamped in enumerate(self.path):
            dist = self.euclidean_distance(self.ego_pose.position, pose_stamped.pose.position)
            if dist < min_dist:
                min_dist = dist
                closest_index = i

        if closest_index is not None and closest_index + 1 < len(self.path):
            return self.path[closest_index + 1].pose
        elif closest_index == len(self.path) - 1:
            self.reached_end = True
            return None
        return None

    @staticmethod
    def euclidean_distance(p1, p2):
        return math.sqrt((p1.x - p2.x) ** 2 + (p1.y - p2.y) ** 2)


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
