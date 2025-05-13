import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Path
from std_msgs.msg import Bool, String
import time


class DummyPublisher(Node):
    def __init__(self):
        super().__init__('dummy_publisher')

        # Publishers
        self.pose_pub = self.create_publisher(PoseStamped, '/ego_pose', 10)
        self.twist_pub = self.create_publisher(TwistStamped, '/ego_twist', 10)
        self.path_pub = self.create_publisher(Path, '/path_data', 10)
        self.obstacle_pub = self.create_publisher(Bool, '/obstacle_detected', 10)
        self.state_pub = self.create_publisher(String, '/vehicle_state', 10)

        # Publish data every second
        self.timer = self.create_timer(1.0, self.publish_dummy_data)

        self.sent_path = False

    def publish_dummy_data(self):
        # Pose
        pose_msg = PoseStamped()
        pose_msg.pose.position.x = 0.0
        pose_msg.pose.position.y = 0.0
        pose_msg.pose.position.z = 0.0
        self.pose_pub.publish(pose_msg)

        # Twist
        twist_msg = TwistStamped()
        twist_msg.twist.linear.x = 0.0
        twist_msg.twist.angular.z = 0.0
        self.twist_pub.publish(twist_msg)

        # Path (just once)
        if not self.sent_path:
            path_msg = Path()
            for i in range(10):
                pose = PoseStamped()
                pose.pose.position.x = float(i)
                pose.pose.position.y = 0.0
                path_msg.poses.append(pose)
            self.path_pub.publish(path_msg)
            self.sent_path = True
            self.get_logger().info('Published dummy path.')

        # Obstacle
        obstacle_msg = Bool()
        obstacle_msg.data = False
        self.obstacle_pub.publish(obstacle_msg)

        # Vehicle State
        state_msg = String()
        state_msg.data = "Driving"
        self.state_pub.publish(state_msg)


def main(args=None):
    rclpy.init(args=args)
    dummy_node = DummyPublisher()
    rclpy.spin(dummy_node)
    dummy_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
