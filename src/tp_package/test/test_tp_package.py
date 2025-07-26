import pytest
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Path
from ackermann_msgs.msg import AckermannDrive

from tp_package.tp_planner import TrajectoryPlanner  # Ensure this import works


@pytest.fixture(scope='session', autouse=True)
def init_rclpy():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def test_node():
    node = TrajectoryPlanner()
    yield node
    node.destroy_node()


@pytest.fixture
def test_subscriber_node():
    class SubscriberNode(Node):
        def __init__(self):
            super().__init__('test_subscriber_node')
            self.received_drive = None
            self.received_location = None

            self.drive_sub = self.create_subscription(
                AckermannDrive,
                '/ackermann_drive',
                self.drive_callback,
                10
            )
            self.location_sub = self.create_subscription(
                String,
                '/student_location',
                self.location_callback,
                10
            )

        def drive_callback(self, msg):
            self.received_drive = msg

        def location_callback(self, msg):
            self.received_location = msg

    subscriber_node = SubscriberNode()
    yield subscriber_node
    subscriber_node.destroy_node()


# Helper functions
def publish_pose(node, x, y):
    msg = PoseStamped()
    msg.pose.position.x = x
    msg.pose.position.y = y
    node.pose_callback(msg)


def publish_twist(node, speed=0.0):
    msg = TwistStamped()
    msg.twist.linear.x = speed
    node.twist_callback(msg)


def publish_path(node):
    path = Path()
    for i in range(5):
        pose = PoseStamped()
        pose.pose.position.x = i * 1.0
        pose.pose.position.y = i * 0.5
        path.poses.append(pose)
    node.path_callback(path)


def publish_obstacle(node, detected=True):
    msg = Bool()
    msg.data = detected
    node.obstacle_callback(msg)


def publish_vehicle_state(node, state):
    msg = String()
    msg.data = state
    node.state_callback(msg)


# Test Cases
def test_drive_command_published(test_node, test_subscriber_node):
    publish_twist(test_node, 0.0)
    publish_pose(test_node, 0.0, 0.0)
    publish_path(test_node)
    publish_vehicle_state(test_node, "Driving")

    for _ in range(10):
        test_node.control_loop()
        rclpy.spin_once(test_subscriber_node, timeout_sec=0.1)

    assert test_subscriber_node.received_drive is not None
    assert test_subscriber_node.received_drive.speed > 0.0


def test_student_location_published_at_goal(test_node, test_subscriber_node):
    publish_twist(test_node, 0.0)
    publish_pose(test_node, 3.5, 1.0)
    publish_path(test_node)
    publish_vehicle_state(test_node, "Driving")

    for _ in range(10):
        test_node.control_loop()
        rclpy.spin_once(test_subscriber_node, timeout_sec=0.1)

    assert test_subscriber_node.received_location is not None
    assert test_subscriber_node.received_location.data == "First Student"


def test_stops_on_obstacle(test_node, test_subscriber_node):
    publish_twist(test_node, 2.0)
    publish_pose(test_node, 1.0, 0.0)
    publish_path(test_node)
    publish_vehicle_state(test_node, "Driving")
    publish_obstacle(test_node, True)

    for _ in range(5):
        test_node.control_loop()
        rclpy.spin_once(test_subscriber_node, timeout_sec=0.1)

    assert test_subscriber_node.received_drive is not None
    assert test_subscriber_node.received_drive.speed == 0.0


def test_stops_on_boarding_state(test_node, test_subscriber_node):
    publish_twist(test_node, 2.0)
    publish_pose(test_node, 1.0, 0.0)
    publish_path(test_node)
    publish_vehicle_state(test_node, "Boarding")
    publish_obstacle(test_node, False)

    for _ in range(5):
        test_node.control_loop()
        rclpy.spin_once(test_subscriber_node, timeout_sec=0.1)

    assert test_subscriber_node.received_drive is not None
    assert test_subscriber_node.received_drive.speed == 0.0


def test_stops_on_idle_state(test_node, test_subscriber_node):
    publish_twist(test_node, 2.0)
    publish_pose(test_node, 1.0, 0.0)
    publish_path(test_node)
    publish_vehicle_state(test_node, "Idle")
    publish_obstacle(test_node, False)

    for _ in range(5):
        test_node.control_loop()
        rclpy.spin_once(test_subscriber_node, timeout_sec=0.1)

    assert test_subscriber_node.received_drive is not None
    assert test_subscriber_node.received_drive.speed == 0.0