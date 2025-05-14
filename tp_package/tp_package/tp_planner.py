import rclpy
from rclpy.node import Node
import math

from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Path
from ackermann_msgs.msg import AckermannDrive


class TrajectoryPlanner(Node):
    def __init__(self):
        super().__init__('trajectory_planner')

        # Declare parameters
        self.declare_parameter('base_lookahead_distance', 1.0)
        self.declare_parameter('lookahead_gain', 1.5)
        self.declare_parameter('max_speed', 3.0)
        self.declare_parameter('min_speed', 1.0)
        self.declare_parameter('curvature_sensitivity', 50.0)  # Further increase sensitivity to 50.0

        # Load parameters
        self.base_ld = self.get_parameter('base_lookahead_distance').get_parameter_value().double_value
        self.ld_gain = self.get_parameter('lookahead_gain').get_parameter_value().double_value
        self.max_speed = self.get_parameter('max_speed').get_parameter_value().double_value
        self.min_speed = self.get_parameter('min_speed').get_parameter_value().double_value
        self.k = self.get_parameter('curvature_sensitivity').get_parameter_value().double_value  # Increased value

        self.wheelbase = 0.26  # meters

        # Steering angle bounds in degrees
        self.steering_limit_deg = 45.0

        # Vehicle state
        self.current_pose = None
        self.current_velocity = 0.0
        self.path_points = []

        # ROS 2 Interfaces
        self.create_subscription(PoseStamped, '/ego_pose', self.pose_callback, 10)
        self.create_subscription(TwistStamped, '/ego_twist', self.twist_callback, 10)
        self.create_subscription(Path, '/path_data', self.path_callback, 10)
        self.drive_pub = self.create_publisher(AckermannDrive, '/ackermann_drive', 10)

        self.create_timer(0.05, self.control_loop)  # 20 Hz

    def pose_callback(self, msg):
        self.current_pose = msg.pose

    def twist_callback(self, msg):
        self.current_velocity = msg.twist.linear.x

    def path_callback(self, msg):
        self.path_points = [p.pose for p in msg.poses]

    def get_yaw(self, q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def transform_to_vehicle_frame(self, global_point, pose):
        x = pose.position.x
        y = pose.position.y
        yaw = self.get_yaw(pose.orientation)

        dx = global_point.x - x
        dy = global_point.y - y

        local_x = math.cos(-yaw) * dx - math.sin(-yaw) * dy
        local_y = math.sin(-yaw) * dx + math.cos(-yaw) * dy

        return local_x, local_y

    def compute_distance(self, p1, p2):
        return math.hypot(p1.x - p2.x, p1.y - p2.y)

    def find_lookahead_point(self, lookahead_distance):
        if not self.current_pose or not self.path_points:
            return None

        pos = self.current_pose.position
        for point in self.path_points:
            if self.compute_distance(pos, point.position) >= lookahead_distance:
                return point.position
        return None

    def estimate_curvature(self, p0, p1, p2):
        def dist(a, b):
            return math.hypot(a.x - b.x, a.y - b.y)

        a = dist(p0, p1)
        b = dist(p1, p2)
        c = dist(p2, p0)

        s = (a + b + c) / 2.0
        area = math.sqrt(max(s * (s - a) * (s - b) * (s - c), 0.0))  # Heron's formula

        if area == 0 or a * b * c == 0:
            return 0.0

        return (4 * area) / (a * b * c)

    def compute_steering_angle(self, lookahead_point, lookahead_distance):
        lx, ly = self.transform_to_vehicle_frame(lookahead_point, self.current_pose)
        if lx <= 0.0:
            return 0.0
        curvature = (2 * ly) / (lookahead_distance ** 2)
        angle_rad = math.atan(curvature * self.wheelbase)
        angle_deg = math.degrees(angle_rad)

        # Clamp the steering angle to be between -45° and +45°
        angle_deg = max(-45.0, min(45.0, angle_deg))
        return angle_deg

    def compute_adaptive_speed(self, curvature):
        return max(self.min_speed, self.max_speed * math.exp(-self.k * curvature))

    def control_loop(self):
        if self.current_pose is None or len(self.path_points) < 3:
            return

        # Compute dynamic lookahead distance based on current velocity
        lookahead_distance = self.base_ld + self.ld_gain * self.current_velocity
        lookahead = self.find_lookahead_point(lookahead_distance)

        if lookahead is None:
            self.publish_stop()
            return

        try:
            current = self.current_pose.position
            next_idx = self.path_points.index(next(p for p in self.path_points if p.position == lookahead))
            p0 = current
            p1 = lookahead
            p2 = self.path_points[min(next_idx + 5, len(self.path_points) - 1)].position
            curvature = self.estimate_curvature(p0, p1, p2)
        except Exception as e:
            self.get_logger().warn(f"Curvature estimation failed: {e}")
            curvature = 0.0

        # Print the curvature to check its value
        self.get_logger().info(f"Curvature: {curvature:.4f}")

        # Calculate speed and steering angle
        speed = self.compute_adaptive_speed(curvature)
        steering_angle_deg = self.compute_steering_angle(lookahead, lookahead_distance)

        # Create AckermannDrive message
        cmd = AckermannDrive()
        cmd.speed = speed
        cmd.steering_angle = steering_angle_deg  # in degrees
        self.drive_pub.publish(cmd)

        # Print the command to terminal
        self.get_logger().info(f"Ackermann Command => Speed: {cmd.speed:.2f} m/s, Steering Angle: {cmd.steering_angle:.2f} deg")

    def publish_stop(self):
        cmd = AckermannDrive()
        cmd.speed = 0.0
        cmd.steering_angle = 0.0
        self.get_logger().info("Stopping vehicle at final waypoint.")
        self.drive_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
