"""Unit tests for the TrajectoryPlanner component."""

import unittest
from unittest.mock import MagicMock, patch
import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import Bool, String
from ackermann_msgs.msg import AckermannDrive
from geometry_msgs.msg import Pose, PoseStamped, Quaternion, Twist

from tp_package.tp_planner import TrajectoryPlanner  # <-- Corrected import

class TestTrajectoryPlanner(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # Initialize rclpy once for all tests
        rclpy.init(args=None)

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = TrajectoryPlanner()
        # Patch publishers so we can check what would be published
        self.node.drive_pub.publish = MagicMock()
        self.node.student_location_pub.publish = MagicMock()
        self.node.student_reached_pub.publish = MagicMock()
        # Patch logger to check log warnings and info
        self.node.get_logger = MagicMock()
        self.node.get_logger().warn = MagicMock()
        self.node.get_logger().info = MagicMock()

    def tearDown(self):
        self.node.destroy_node()

    def get_pose(self, x=0.0, y=0.0, yaw=0.0):
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.orientation = Quaternion(
            x=0.0,
            y=0.0,
            z=math.sin(yaw / 2.0),
            w=math.cos(yaw / 2.0)
        )
        return pose

    def get_odometry(self, x=0.0, y=0.0, yaw=0.0):
        odom = Odometry()
        odom.pose.pose = self.get_pose(x, y, yaw)
        odom.twist.twist = Twist()
        return odom

    def get_path(self, waypoints):
        path = Path()
        for x, y in waypoints:
            pose_stamped = PoseStamped()
            pose_stamped.pose = self.get_pose(x, y)
            path.poses.append(pose_stamped)
        return path

    # AC1
    def test_missing_inputs_skips_publish_and_warns_once(self):
        self.node.ego_pose = None
        self.node.ego_twist = None
        self.node.path = []
        self.node.control_loop()
        self.assertFalse(self.node.drive_pub.publish.called)
        self.node.get_logger().warn.assert_called_once()
        self.node.control_loop()
        self.node.get_logger().warn.assert_called_once()

    # AC2
    def test_obstacle_detected_publishes_stop(self):
        self.node.ego_pose = self.get_pose()
        self.node.ego_twist = Twist()
        self.node.path = [PoseStamped(pose=self.get_pose(1.0, 0.0))]
        self.node.obstacle_detected = True
        self.node.vehicle_state = "Driving"
        self.node.control_loop()
        msg = self.node.drive_pub.publish.call_args[0][0]
        self.assertEqual(msg.speed, 0.0)
        self.assertEqual(msg.steering_angle, 0.0)

    # AC3
    def test_vehicle_state_not_driving_publishes_stop(self):
        self.node.ego_pose = self.get_pose()
        self.node.ego_twist = Twist()
        self.node.path = [PoseStamped(pose=self.get_pose(1.0, 0.0))]
        self.node.obstacle_detected = False
        self.node.vehicle_state = "Idle"
        self.node.control_loop()
        msg = self.node.drive_pub.publish.call_args[0][0]
        self.assertEqual(msg.speed, 0.0)
        self.assertEqual(msg.steering_angle, 0.0)

    # AC4
    def test_pure_pursuit_command_limits(self):
        self.node.ego_pose = self.get_pose()
        self.node.ego_twist = Twist()
        self.node.path = [PoseStamped(pose=self.get_pose(1.0, 0.0))]
        self.node.obstacle_detected = False
        self.node.vehicle_state = "Driving"
        self.node.max_steering_angle_deg = 10.0
        self.node.steering_offset_deg = 2.0
        self.node.min_speed = 0.1
        self.node.max_speed = 0.5
        self.node.control_loop()
        msg = self.node.drive_pub.publish.call_args[0][0]
        self.assertLessEqual(abs(msg.steering_angle), 10.0)
        self.assertGreaterEqual(msg.speed, 0.1)
        self.assertLessEqual(msg.speed, 0.5)

    # AC5
    def test_endpoint_stop(self):
        self.node.ego_pose = self.get_pose(0.0, 0.0)
        self.node.ego_twist = Twist()
        self.node.path = [PoseStamped(pose=self.get_pose(0.1, 0.0))]
        self.node.obstacle_detected = False
        self.node.vehicle_state = "Driving"
        self.node.control_loop()
        msg = self.node.drive_pub.publish.call_args[0][0]
        self.assertEqual(msg.speed, 0.0)
        self.assertEqual(msg.steering_angle, 0.0)

    # AC6
    def test_print_ackermann_drive_in_terminal(self):
        self.node.ego_pose = self.get_pose()
        self.node.ego_twist = Twist()
        self.node.path = [PoseStamped(pose=self.get_pose(1.0, 0.0))]
        self.node.obstacle_detected = False
        self.node.vehicle_state = "Driving"
        self.node.control_loop()
        self.node.get_logger().info.assert_any_call(
            "[DRIVE] speed: %.2f m/s | steering_angle: %.2f deg",
            self.node.drive_pub.publish.call_args[0][0].speed,
            self.node.drive_pub.publish.call_args[0][0].steering_angle
        )

    # AC7: Prune path removes behind waypoints
    def test_prune_path_removes_behind_waypoints(self):
        self.node.ego_pose = self.get_pose(0.0, 0.0, 0.0)
        self.node.path = [
            PoseStamped(pose=self.get_pose(-1.0, 0.0)),
            PoseStamped(pose=self.get_pose(0.5, 0.0)),
            PoseStamped(pose=self.get_pose(1.0, 0.0))
        ]
        self.node.prune_path()
        self.assertEqual(len(self.node.path), 2)
        self.assertAlmostEqual(self.node.path[0].pose.position.x, 0.5)

    # AC8: Feedback callback
    def test_feedback_callback_noop(self):
        msg = AckermannDrive()
        self.node.feedback_callback(msg)  # No-op but must not error

    # AC9: All path behind logs and stops
    def test_all_path_behind_stops_and_warns(self):
        self.node.ego_pose = self.get_pose(5.0, 0.0, 0.0)
        self.node.ego_twist = Twist()
        self.node.path = [PoseStamped(pose=self.get_pose(0.0, 0.0))]
        self.node.vehicle_state = "Driving"
        self.node.obstacle_detected = False
        self.node.control_loop()
        self.node.get_logger().warn.assert_called_with(
            "All path points are behind the vehicle. Stopping and requesting new path."
        )
        msg = self.node.drive_pub.publish.call_args[0][0]
        self.assertEqual(msg.speed, 0.0)

    # AC10: Student location publishing
    def test_student_location_publishing(self):
        self.node.ego_pose = self.get_pose(1.0, 0.0)
        self.node.ego_twist = Twist()
        self.node.path = [PoseStamped(pose=self.get_pose(1.1, 0.0))]
        self.node.vehicle_state = "Driving"
        self.node.obstacle_detected = False
        self.node.student_publish_index = 0
        self.node.ready_to_publish_student = True
        self.node.student_published_for_this_path = False
        self.node.was_at_endpoint = False
        self.node.control_loop()
        self.node.student_location_pub.publish.assert_called()
        self.node.student_reached_pub.publish.assert_called()

    # AC11: No lookahead point found fallback
    def test_no_lookahead_fallback_uses_closest(self):
        self.node.ego_pose = self.get_pose(0.0, 0.0, 0.0)
        self.node.path = [
            PoseStamped(pose=self.get_pose(0.5, 0.0)),
            PoseStamped(pose=self.get_pose(0.6, 0.0))
        ]
        steer, saturated = self.node.pure_pursuit_steering()
        self.assertIsInstance(steer, float)
        self.assertFalse(saturated)

if __name__ == '__main__':
    unittest.main()
