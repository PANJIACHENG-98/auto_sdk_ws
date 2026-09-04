"""Validate ROS runtime ownership and generated request-field translation."""

import math

import pytest

from geometry_msgs.msg import TransformStamped
from rclpy.context import Context
from rclpy.node import Node

from wheelloong_auto_sdk import (
    CartesianPoseMM,
    CommandResult,
    MotionMode,
    NavigationMode,
    NavigationPose,
    Robot,
)
from wheelloong_auto_sdk.backend.ros2 import ROS2Backend


def test_standalone_owns_and_closes_private_runtime():
    """Close the private Node, executor and Context owned by standalone mode."""
    robot = Robot.standalone(node_name="sdk_standalone_lifecycle_test")
    assert robot.node.get_name() == "sdk_standalone_lifecycle_test"
    robot.close()


def test_from_node_does_not_destroy_caller_node_or_context():
    """Leave a borrowed Node and its caller-owned Context alive on close."""
    context = Context()
    context.init()
    node = Node("sdk_borrowed_node_test", context=context)
    robot = Robot.from_node(node)
    robot.close()
    assert context.ok()
    assert node.get_name() == "sdk_borrowed_node_test"
    node.destroy_node()
    context.shutdown()


def test_real_generated_ros_request_translation_without_sending_commands():
    """Inspect generated service requests without contacting robot services."""
    backend = ROS2Backend.standalone(node_name="sdk_request_translation_test")
    captured = []

    def capture(key, request, operation, timeout_sec):
        """Record a generated request in place of the real ROS service call."""
        captured.append((key, request, operation, timeout_sec))
        return CommandResult(operation)

    backend._service_call = capture
    pose = CartesianPoseMM(400.0, 100.0, 200.0)
    joints = (0.0,) * 7
    try:
        backend.move_j(joints, joints, MotionMode.ABSOLUTE, 0.3, 0.4, False, 5.0)
        assert captured[-1][1].robot_index == -1
        assert captured[-1][1].speed_left.move_j_vel == pytest.approx(0.3)
        assert captured[-1][1].speed_left.move_j_acc == pytest.approx(0.4)
        assert list(captured[-1][1].effort_left) == [0.0] * 7
        assert list(captured[-1][1].effort_right) == [0.0] * 7
        assert captured[-1][1].zone_left == 0
        assert captured[-1][1].zone_right == 0

        backend.move_l(
            pose,
            None,
            MotionMode.INCREMENTAL,
            200.0,
            300.0,
            0.0,
            0.0,
            False,
            5.0,
        )
        assert captured[-1][1].mode_left == 1
        assert captured[-1][1].pose_left.pose.position.x == 400.0

        backend.move_p(pose, None, 0.3, 0.4, joints, None, False, 5.0)
        assert list(captured[-1][1].ref_joint_left) == list(joints)

        backend.move_body(None, 0.2, None, -100.0, 50.0, False, 5.0)
        assert list(captured[-1][1].index) == [False, True, False, True]
        assert list(captured[-1][1].value) == pytest.approx([0.0, 0.2, 0.0, -100.0])

        nav_pose = backend._navigation_pose_message(NavigationPose(1.0, 2.0, 0.0))
        assert nav_pose.header.frame_id == "map"
        assert nav_pose.pose.orientation.w == 1.0

        backend.clear_local_costmap(4.0)
        assert captured[-1][0] == "clear_local_costmap"
        assert captured[-1][2:] == ("clear local costmap", 4.0)
        backend.clear_global_costmap(6.0)
        assert captured[-1][0] == "clear_global_costmap"
        assert captured[-1][2:] == ("clear global costmap", 6.0)
    finally:
        backend.close()


def test_system_info_quaternion_pose_translation():
    """将 /system/get_info 的 xyz+xyzw 转为公共 TCP 位姿。"""
    pose = ROS2Backend._system_pose(
        [1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 2.0], "arm_base"
    )
    assert pose == CartesianPoseMM(1.0, 2.0, 3.0, frame_id="arm_base")
    assert ROS2Backend._system_pose([0.0] * 7, "arm_base") is None


def test_navigation_tf_translation():
    """Convert map-to-base translation and quaternion into a public pose."""
    transform = TransformStamped()
    transform.transform.translation.x = 1.25
    transform.transform.translation.y = -0.4
    transform.transform.translation.z = 0.1
    transform.transform.rotation.z = math.sin(math.pi / 4.0)
    transform.transform.rotation.w = math.cos(math.pi / 4.0)

    pose = ROS2Backend._navigation_pose_from_transform(transform, "map")

    assert pose.x_m == pytest.approx(1.25)
    assert pose.y_m == pytest.approx(-0.4)
    assert pose.z_m == pytest.approx(0.1)
    assert pose.yaw_rad == pytest.approx(math.pi / 2.0)
    assert pose.frame_id == "map"


def test_navigation_tree_ignores_product_model_environment(monkeypatch):
    """Do not turn WLOONG product metadata into an invalid BT path."""
    monkeypatch.setenv("ROBOT_NAV_VERSION", "WLOONG")

    assert ROS2Backend._behavior_tree(
        NavigationMode.DEFAULT, "", through=False
    ) == ""
    assert ROS2Backend._behavior_tree(
        NavigationMode.DEFAULT, "", through=True
    ) == ""

    monkeypatch.setenv("ROBOT_NAV_VERSION", "sloong_nav")
    assert ROS2Backend._behavior_tree(
        NavigationMode.DEFAULT, "", through=True
    ) == "sloong_nav/navigate_through_poses.xml"
