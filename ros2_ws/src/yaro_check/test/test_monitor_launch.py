"""Integration test: start joint_limit_monitor with the YARO-1105 URDF and talk to it over DDS.

Checks the whole path a real controller would use: /joint_states in, /diagnostics,
~/limits_ok and ~/tip_pose out.
"""
import math
import os
import time
import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

URDF = os.path.join(os.environ["YARO_MODELS_DIR"], "yaro_1105", "robot.urdf")
JOINTS = [f"joint_{i}" for i in range(1, 7)]


@pytest.mark.launch_test
def generate_test_description():
    with open(URDF) as f:
        description = f.read()
    node = launch_ros.actions.Node(
        package="yaro_check",
        executable="joint_limit_monitor",
        name="joint_limit_monitor",
        parameters=[{"robot_description": description, "warn_margin_rad": 0.05}],
        output="screen",
    )
    return launch.LaunchDescription([node, launch_testing.actions.ReadyToTest()]), {"monitor": node}


class TestJointLimitMonitor(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("monitor_test_client")
        self.ok, self.diag, self.pose = [], [], []
        self.node.create_subscription(Bool, "/joint_limit_monitor/limits_ok", self.ok.append, 10)
        self.node.create_subscription(DiagnosticArray, "/diagnostics", self.diag.append, 10)
        self.node.create_subscription(PoseStamped, "/joint_limit_monitor/tip_pose", self.pose.append, 10)
        self.pub = self.node.create_publisher(JointState, "/joint_states", 10)
        deadline = time.time() + 10.0
        while self.pub.get_subscription_count() == 0 and time.time() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.assertGreater(self.pub.get_subscription_count(), 0, "monitor never subscribed to /joint_states")

    def tearDown(self):
        self.node.destroy_node()

    def send_until_reply(self, positions, velocities=None, names=None, timeout=10.0):
        """Publish one JointState repeatedly until a fresh /diagnostics reply arrives."""
        self.ok.clear()
        self.diag.clear()
        self.pose.clear()
        msg = JointState()
        msg.name = names or JOINTS
        msg.position = [float(p) for p in positions]
        msg.velocity = [float(v) for v in (velocities or [])]
        deadline = time.time() + timeout
        while time.time() < deadline:
            msg.header.stamp = self.node.get_clock().now().to_msg()
            self.pub.publish(msg)
            end = time.time() + 0.2
            while time.time() < end:
                rclpy.spin_once(self.node, timeout_sec=0.05)
            if self.diag and self.ok:
                return self.diag[-1], self.ok[-1].data
        self.fail("no reply from joint_limit_monitor")

    def levels(self, diag):
        # DiagnosticStatus.level is a ROS `byte`, which rclpy hands back as a 1-byte bytes object.
        return {s.name.split(": ")[1]: int.from_bytes(s.level, "little") for s in diag.status}

    def test_home_pose_is_ok_and_publishes_tip_pose(self):
        diag, ok = self.send_until_reply([0.0] * 6, [0.0] * 6)
        self.assertTrue(ok)
        self.assertEqual(set(self.levels(diag).values()), {0})
        self.assertTrue(self.pose, "no tip pose published")
        p = self.pose[-1].pose.position
        self.assertEqual(self.pose[-1].header.frame_id, "link_0")
        # Home pose of the YARO-1105 URDF: arm stretched upward, tip 1.41 m above the base.
        self.assertAlmostEqual(math.hypot(p.x, p.y), 0.0, delta=0.2)
        self.assertGreater(p.z, 1.0)

    def test_position_beyond_limit_is_an_error(self):
        diag, ok = self.send_until_reply([0.0, 0.0, 2.9, 0.0, 0.0, 0.0], [0.0] * 6)  # joint_3 limit is 2.79 rad
        self.assertFalse(ok)
        self.assertEqual(self.levels(diag)["joint_3"], 2)

    def test_close_to_limit_is_a_warning(self):
        diag, ok = self.send_until_reply([0.0, 0.0, 2.77, 0.0, 0.0, 0.0], [0.0] * 6)
        self.assertTrue(ok)
        self.assertEqual(self.levels(diag)["joint_3"], 1)

    def test_velocity_above_limit_is_an_error(self):
        diag, ok = self.send_until_reply([0.0] * 6, [0.0, 0.0, 0.0, 0.0, 0.0, 8.0])  # joint_6 limit 7.33 rad/s
        self.assertFalse(ok)
        self.assertEqual(self.levels(diag)["joint_6"], 2)

    def test_missing_joint_is_an_error(self):
        diag, ok = self.send_until_reply([0.0] * 5, [], names=JOINTS[:5])
        self.assertFalse(ok)
        self.assertEqual(self.levels(diag)["joint_6"], 2)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    def test_exit_code(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -2, -15])
