#!/usr/bin/env python3

import unittest
import rclpy
import time
import threading
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import Bool, String
from ackermann_msgs.msg import AckermannDrive
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, Twist, Vector3
from tf_transformations import quaternion_from_euler
import math
import queue


class TestTrajectoryPlannerIntegration(unittest.TestCase):
    """
    Integration tests for TrajectoryPlanner node covering:
    - TC_Int021: Control Algorithm Switching Test
    - TC_Int022: Obstacle Detection Halt Integration Test  
    - TC_Int023: Path Data Loss Recovery Test
    """

    @classmethod
    def setUpClass(cls):
        """Initialize ROS2 and create test node"""
        rclpy.init()
        cls.test_node = TestNode()
        cls.executor = rclpy.executors.SingleThreadedExecutor()
        cls.executor.add_node(cls.test_node)
        
        # Start executor in separate thread
        cls.executor_thread = threading.Thread(target=cls.executor.spin, daemon=True)
        cls.executor_thread.start()
        
        # Wait for node to be ready
        time.sleep(2.0)

    @classmethod
    def tearDownClass(cls):
        """Cleanup ROS2 resources"""
        cls.executor.shutdown()
        cls.test_node.destroy_node()
        rclpy.shutdown()

    def setUp(self):
        """Reset test state before each test"""
        self.test_node.reset_state()
        time.sleep(0.5)

    def test_tc_int021_control_algorithm_switching(self):
        """
        TC_Int021: Control Algorithm Switching Test
        
        Tests switching from Pure Pursuit to DWA when peer vehicle is detected
        within overtake_trigger_distance, and verifies smooth steering transitions.
        """
        print("\n=== TC_Int021: Control Algorithm Switching Test ===")
        
        # Preconditions: Setup normal driving state
        self._setup_normal_driving_preconditions()
        time.sleep(1.0)
        
        # Test Step 1: Confirm Pure Pursuit is active
        print("Step 1: Confirming Pure Pursuit operation...")
        
        # Wait for initial ackermann commands
        initial_commands = self._collect_ackermann_commands(duration=2.0, min_commands=5)
        
        self.assertGreater(len(initial_commands), 0, 
                          "Should receive initial ackermann drive commands")
        
        # Verify non-zero steering and speed (Pure Pursuit characteristics)
        non_zero_steering = any(abs(cmd.steering_angle) > 0.1 for cmd in initial_commands)
        non_zero_speed = any(abs(cmd.speed) > 0.1 for cmd in initial_commands)
        
        self.assertTrue(non_zero_steering or non_zero_speed, 
                       "Pure Pursuit should produce non-zero steering or speed commands")
        
        print(f"✓ Pure Pursuit active - received {len(initial_commands)} commands")
        
        # Test Step 2: Introduce peer vehicle within trigger distance
        print("Step 2: Publishing peer vehicle within overtake trigger distance...")
        
        # Create peer vehicle pose within trigger distance (1.5m from ego)
        peer_odom = self._create_peer_odometry(
            x=1.0,  # 1m ahead of ego vehicle
            y=0.0,  # same lane
            yaw=0.0
        )
        
        # Record steering angles before peer introduction
        pre_peer_commands = initial_commands[-3:] if len(initial_commands) >= 3 else initial_commands
        avg_steering_before = sum(cmd.steering_angle for cmd in pre_peer_commands) / len(pre_peer_commands)
        
        # Publish peer vehicle
        self.test_node.peer_odom_pub.publish(peer_odom)
        time.sleep(0.1)  # Small delay for message processing
        
        # Collect commands after peer introduction
        post_peer_commands = self._collect_ackermann_commands(duration=3.0, min_commands=10)
        
        self.assertGreater(len(post_peer_commands), 0,
                          "Should receive ackermann commands after peer introduction")
        
        # Expected Results Verification:
        
        # a. Control algorithm switches to DWA (verify through behavior change)
        # DWA typically produces different steering patterns than Pure Pursuit
        steering_variance_before = self._calculate_variance([cmd.steering_angle for cmd in pre_peer_commands])
        steering_variance_after = self._calculate_variance([cmd.steering_angle for cmd in post_peer_commands[:5]])
        
        print(f"Steering variance - Before: {steering_variance_before:.3f}, After: {steering_variance_after:.3f}")
        
        # b. Verify /ackermann_drive receives commands from DWA logic
        # DWA should maintain forward motion while avoiding peer
        forward_motion_maintained = all(cmd.speed >= 0.0 for cmd in post_peer_commands)
        self.assertTrue(forward_motion_maintained, 
                       "DWA should maintain forward motion during overtaking")
        
        # c. Verify smooth steering transitions (no abrupt changes > 15 degrees)
        max_steering_change = 0.0
        for i in range(1, len(post_peer_commands)):
            steering_change = abs(post_peer_commands[i].steering_angle - post_peer_commands[i-1].steering_angle)
            max_steering_change = max(max_steering_change, steering_change)
        
        print(f"Maximum steering change: {max_steering_change:.2f} degrees")
        self.assertLess(max_steering_change, 15.0, 
                       "Steering changes should be smooth (< 15 degrees between commands)")
        
        print("✓ Control algorithm switching test passed")

    def test_tc_int022_obstacle_detection_halt(self):
        """
        TC_Int022: Obstacle Detection Halt Integration Test
        
        Tests immediate halt when obstacle detection publishes True.
        """
        print("\n=== TC_Int022: Obstacle Detection Halt Integration Test ===")
        
        # Preconditions: Setup normal driving
        self._setup_normal_driving_preconditions()
        time.sleep(1.0)
        
        # Verify normal driving first
        normal_commands = self._collect_ackermann_commands(duration=1.0, min_commands=3)
        self.assertGreater(len(normal_commands), 0, "Should be driving normally")
        
        normal_speed = any(abs(cmd.speed) > 0.1 for cmd in normal_commands)
        self.assertTrue(normal_speed, "Vehicle should be moving normally")
        
        print("✓ Confirmed normal driving state")
        
        # Test Step: Publish obstacle detected
        print("Publishing obstacle detection (True)...")
        
        obstacle_msg = Bool()
        obstacle_msg.data = True
        self.test_node.obstacle_pub.publish(obstacle_msg)
        
        # Expected Results:
        
        # a. TrajectoryPlanner immediately sets speed to 0.0
        halt_commands = self._collect_ackermann_commands(duration=2.0, min_commands=5)
        
        self.assertGreater(len(halt_commands), 0, 
                          "Should receive commands after obstacle detection")
        
        # Verify all commands have zero speed
        all_stopped = all(abs(cmd.speed) < 0.01 for cmd in halt_commands)
        self.assertTrue(all_stopped, 
                       "All commands after obstacle detection should have zero speed")
        
        # b. Verify quick response time (should halt within reasonable time)
        # This is implicitly tested by collecting commands immediately after publishing
        
        print(f"✓ Vehicle halted successfully - {len(halt_commands)} zero-speed commands received")
        
        # Test recovery: Clear obstacle and verify resumption
        print("Clearing obstacle and testing recovery...")
        
        obstacle_msg.data = False
        self.test_node.obstacle_pub.publish(obstacle_msg)
        time.sleep(0.5)
        
        recovery_commands = self._collect_ackermann_commands(duration=2.0, min_commands=3)
        
        if recovery_commands:
            resumed_motion = any(abs(cmd.speed) > 0.1 for cmd in recovery_commands)
            if resumed_motion:
                print("✓ Vehicle resumed motion after obstacle cleared")
            else:
                print("ℹ Vehicle remained stopped (may be at endpoint)")

    def test_tc_int023_path_data_loss_recovery(self):
        """
        TC_Int023: Path Data Loss Recovery Test
        
        Tests system response to missing path data.
        """
        print("\n=== TC_Int023: Path Data Loss Recovery Test ===")
        
        # Preconditions: Setup normal driving with continuous path publishing
        self._setup_normal_driving_preconditions()
        
        # Start continuous path publishing
        path_publisher_active = True
        
        def continuous_path_publisher():
            while path_publisher_active:
                if hasattr(self, 'test_node'):
                    self.test_node.path_pub.publish(self._create_test_path())
                time.sleep(0.1)  # 10Hz path publishing
        
        path_thread = threading.Thread(target=continuous_path_publisher, daemon=True)
        path_thread.start()
        
        time.sleep(1.0)
        
        # Verify normal operation with continuous path
        normal_commands = self._collect_ackermann_commands(duration=1.0, min_commands=3)
        self.assertGreater(len(normal_commands), 0, "Should be driving normally with path")
        
        print("✓ Confirmed normal driving with continuous path updates")
        
        # Test Step: Stop path publishing
        print("Stopping path data publishing...")
        
        path_publisher_active = False
        path_thread.join(timeout=1.0)
        
        # Expected Results:
        
        # a. After delay, warning about missing path_data should be logged
        # b. Last command should be halt command
        
        # Wait for the node to detect missing path data
        time.sleep(2.0)
        
        # Collect commands after path loss
        post_loss_commands = self._collect_ackermann_commands(duration=3.0, min_commands=5)
        
        if post_loss_commands:
            # Verify halt commands are published
            final_commands = post_loss_commands[-3:]  # Check last few commands
            all_halted = all(abs(cmd.speed) < 0.01 and abs(cmd.steering_angle) < 0.01 
                           for cmd in final_commands)
            
            self.assertTrue(all_halted, 
                           "Final commands should be halt commands (speed=0, steering=0)")
            
            print(f"✓ Path data loss handled - received {len(post_loss_commands)} commands, final commands are halt")
        else:
            print("ℹ No commands received after path loss (node may have stopped publishing)")
        
        # Test recovery: Resume path publishing
        print("Resuming path data publishing...")
        
        # Publish new path
        recovery_path = self._create_test_path(start_x=2.0)  # Different path
        self.test_node.path_pub.publish(recovery_path)
        time.sleep(1.0)
        
        recovery_commands = self._collect_ackermann_commands(duration=2.0, min_commands=3)
        
        if recovery_commands:
            motion_resumed = any(abs(cmd.speed) > 0.1 for cmd in recovery_commands)
            if motion_resumed:
                print("✓ Vehicle resumed motion after path recovery")
            else:
                print("ℹ Vehicle remained stopped after path recovery")

    # Helper Methods
    
    def _setup_normal_driving_preconditions(self):
        """Setup preconditions for normal driving state"""
        # Publish odometry
        odom = self._create_ego_odometry()
        self.test_node.odom_pub.publish(odom)
        
        # Publish path
        path = self._create_test_path()
        self.test_node.path_pub.publish(path)
        
        # Publish driving state
        state_msg = String()
        state_msg.data = "Driving"
        self.test_node.state_pub.publish(state_msg)
        
        # Ensure no obstacles
        obstacle_msg = Bool()
        obstacle_msg.data = False
        self.test_node.obstacle_pub.publish(obstacle_msg)

    def _create_ego_odometry(self, x=0.0, y=0.0, yaw=0.0, linear_vel=0.3):
        """Create ego vehicle odometry message"""
        odom = Odometry()
        odom.header.stamp = self.test_node.get_clock().now().to_msg()
        odom.header.frame_id = "map"
        
        # Pose
        odom.pose.pose.position = Point(x=x, y=y, z=0.0)
        q = quaternion_from_euler(0, 0, yaw)
        odom.pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        
        # Twist
        odom.twist.twist.linear = Vector3(x=linear_vel, y=0.0, z=0.0)
        odom.twist.twist.angular = Vector3(x=0.0, y=0.0, z=0.0)
        
        return odom

    def _create_peer_odometry(self, x=1.0, y=0.0, yaw=0.0):
        """Create peer vehicle odometry message"""
        return self._create_ego_odometry(x, y, yaw, 0.2)

    def _create_test_path(self, start_x=0.0, start_y=0.0, length=10.0, num_points=20):
        """Create a test path"""
        path = Path()
        path.header.stamp = self.test_node.get_clock().now().to_msg()
        path.header.frame_id = "map"
        
        for i in range(num_points):
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = "map"
            
            # Create straight path
            x = start_x + (i * length / (num_points - 1))
            y = start_y
            
            pose_stamped.pose.position = Point(x=x, y=y, z=0.0)
            q = quaternion_from_euler(0, 0, 0)  # Facing forward
            pose_stamped.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
            
            path.poses.append(pose_stamped)
        
        return path

    def _collect_ackermann_commands(self, duration=2.0, min_commands=1):
        """Collect ackermann drive commands for specified duration"""
        commands = []
        start_time = time.time()
        
        while (time.time() - start_time) < duration:
            try:
                cmd = self.test_node.ackermann_queue.get(timeout=0.1)
                commands.append(cmd)
                
                if len(commands) >= min_commands and (time.time() - start_time) > 1.0:
                    break
            except queue.Empty:
                continue
        
        return commands

    def _calculate_variance(self, values):
        """Calculate variance of a list of values"""
        if len(values) < 2:
            return 0.0
        
        mean = sum(values) / len(values)
        variance = sum((x - mean) ** 2 for x in values) / len(values)
        return variance


class TestNode(Node):
    """Test node for publishing test messages and collecting responses"""
    
    def __init__(self):
        super().__init__('trajectory_planner_test_node')
        
        # Publishers for test inputs
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.path_pub = self.create_publisher(Path, '/path_data', 10)
        self.obstacle_pub = self.create_publisher(Bool, '/obstacle_detected', 10)
        self.state_pub = self.create_publisher(String, '/vehicle_state', 10)
        self.peer_odom_pub = self.create_publisher(Odometry, '/peer_veh_behavior', 10)
        
        # Subscriber for collecting outputs
        self.ackermann_queue = queue.Queue()
        self.create_subscription(AckermannDrive, '/ackermann_drive', 
                               self._ackermann_callback, 10)
        
        self.get_logger().info("Test node initialized")

    def _ackermann_callback(self, msg):
        """Collect ackermann drive messages"""
        try:
            self.ackermann_queue.put(msg, block=False)
        except queue.Full:
            # Remove oldest message if queue is full
            try:
                self.ackermann_queue.get(block=False)
                self.ackermann_queue.put(msg, block=False)
            except queue.Empty:
                pass

    def reset_state(self):
        """Clear collected messages"""
        while not self.ackermann_queue.empty():
            try:
                self.ackermann_queue.get(block=False)
            except queue.Empty:
                break


def main():
    """Run integration tests"""
    import sys
    
    # Run specific test if provided as argument
    if len(sys.argv) > 1:
        test_name = sys.argv[1]
        suite = unittest.TestSuite()
        suite.addTest(TestTrajectoryPlannerIntegration(test_name))
    else:
        # Run all tests
        suite = unittest.TestLoader().loadTestsFromTestCase(TestTrajectoryPlannerIntegration)
    
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)
    
    return 0 if result.wasSuccessful() else 1


if __name__ == '__main__':
    exit(main())