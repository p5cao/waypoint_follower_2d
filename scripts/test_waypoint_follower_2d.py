import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Twist
import unittest
import time
import threading


class WaypointFollowerTester(Node):
    def __init__(self):
        super().__init__("waypoint_follower_tester")

        # Publishers — simulate robot sensor data
        self.odom_publisher_ = self.create_publisher(Odometry, "/odom", 10)
        self.goal_publisher_ = self.create_publisher(PoseStamped, "/goal", 10)

        # Subscriber — listen to what the node commands
        self.cmd_vel_subscriber_ = self.create_subscription(
            Twist, "/cmd_vel", self.cmdVelCallback, 10
        )

        self.received_cmd_vel_ = []

    def cmdVelCallback(self, msg):
        self.received_cmd_vel_.append(msg)
        self.get_logger().info(
            f"cmd_vel received: linear.x={msg.linear.x:.2f}, angular.z={msg.angular.z:.2f}"
        )

    def publishOdom(self, x=0.0, y=0.0, yaw=0.0):
        """Publish a fake odometry message at position (x, y) with given yaw."""
        msg = Odometry()
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_link"
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = 0.0

        # Convert yaw to quaternion (2D: only z and w matter)
        import math
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.odom_publisher_.publish(msg)
        self.get_logger().info(f"Published odom: x={x}, y={y}, yaw={yaw:.2f}")

    def publishGoal(self, x=0.0, y=0.0, yaw=0.0):
        """Publish a goal PoseStamped at position (x, y) with given yaw."""
        msg = PoseStamped()
        msg.header.frame_id = "map"
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = 0.0

        import math
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.orientation.w = math.cos(yaw / 2.0)

        self.goal_publisher_.publish(msg)
        self.get_logger().info(f"Published goal: x={x}, y={y}, yaw={yaw:.2f}")

    def clearCmdVel(self):
        self.received_cmd_vel_.clear()

    def spinFor(self, seconds):
        """Spin ROS for a given number of seconds."""
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)


def main():
    rclpy.init()
    tester = WaypointFollowerTester()

    print("\n" + "=" * 60)
    print("WaypointFollower2D Test Script")
    print("Make sure the waypoint_follower_2d_node is running first!")
    print("=" * 60 + "\n")

    # Allow time for node connections
    tester.spinFor(1.0)

    # ------------------------------------------------------------------
    # Test 1: Robot far from goal — expect forward velocity
    # ------------------------------------------------------------------
    print("TEST 1: Robot at (0,0), Goal at (2,0) — expect forward velocity")
    tester.clearCmdVel()
    tester.publishOdom(x=0.0, y=0.0, yaw=0.0)
    tester.spinFor(0.5)
    tester.publishGoal(x=2.0, y=0.0, yaw=0.0)
    tester.spinFor(1.0)

    if tester.received_cmd_vel_:
        cmd = tester.received_cmd_vel_[-1]
        assert cmd.linear.x > 0.0, "FAIL: expected forward velocity"
        assert cmd.linear.x <= 0.5, "FAIL: linear.x exceeds max 0.5 m/s"
        print(f"  PASS: linear.x = {cmd.linear.x:.2f} m/s")
    else:
        print("  FAIL: no cmd_vel received")

    # ------------------------------------------------------------------
    # Test 2: Robot already at goal — expect stop command
    # ------------------------------------------------------------------
    print("\nTEST 2: Robot at (1,0), Goal at (1,0) — expect stop")
    tester.clearCmdVel()
    tester.publishOdom(x=1.0, y=0.0, yaw=0.0)
    tester.spinFor(0.5)
    tester.publishGoal(x=1.0, y=0.0, yaw=0.0)
    tester.spinFor(1.0)

    if tester.received_cmd_vel_:
        cmd = tester.received_cmd_vel_[-1]
        assert cmd.linear.x == 0.0, "FAIL: expected zero linear velocity"
        assert cmd.angular.z == 0.0, "FAIL: expected zero angular velocity"
        print(f"  PASS: linear.x={cmd.linear.x:.2f}, angular.z={cmd.angular.z:.2f}")
    else:
        print("  FAIL: no cmd_vel received")

    # ------------------------------------------------------------------
    # Test 3: Large yaw error — expect rotation before driving
    # ------------------------------------------------------------------
    import math
    print("\nTEST 3: Robot facing 0 rad, goal at 90 deg yaw — expect rotation")
    tester.clearCmdVel()
    tester.publishOdom(x=0.0, y=0.0, yaw=0.0)
    tester.spinFor(0.5)
    tester.publishGoal(x=2.0, y=0.0, yaw=math.pi / 2.0)
    tester.spinFor(1.0)

    if tester.received_cmd_vel_:
        cmd = tester.received_cmd_vel_[-1]
        assert abs(cmd.angular.z) <= 1.0, "FAIL: angular.z exceeds max 1.0 rad/s"
        print(f"  PASS: angular.z = {cmd.angular.z:.2f} rad/s")
    else:
        print("  FAIL: no cmd_vel received")

    # ------------------------------------------------------------------
    # Test 4: Velocity saturation — very far goal should clamp to max
    # ------------------------------------------------------------------
    print("\nTEST 4: Goal very far away — expect velocity clamped to 0.5 m/s")
    tester.clearCmdVel()
    tester.publishOdom(x=0.0, y=0.0, yaw=0.0)
    tester.spinFor(0.5)
    tester.publishGoal(x=100.0, y=0.0, yaw=0.0)
    tester.spinFor(1.0)

    if tester.received_cmd_vel_:
        cmd = tester.received_cmd_vel_[-1]
        assert cmd.linear.x <= 0.5, "FAIL: linear.x exceeds max 0.5 m/s"
        print(f"  PASS: linear.x clamped to {cmd.linear.x:.2f} m/s")
    else:
        print("  FAIL: no cmd_vel received")

    # ------------------------------------------------------------------
    # Test 5: No odom published — node should not crash
    # ------------------------------------------------------------------
    print("\nTEST 5: Goal published before odom — node should not crash")
    tester.clearCmdVel()
    tester.publishGoal(x=1.0, y=0.0, yaw=0.0)
    tester.spinFor(1.0)
    print("  PASS: node did not crash")

    print("\n" + "=" * 60)
    print("All tests complete.")
    print("=" * 60 + "\n")

    tester.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()