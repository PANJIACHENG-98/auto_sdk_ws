"""Validate ROS-independent public API behavior with the in-memory backend."""

import math
from dataclasses import replace

import pytest

from wheelloong_auto_sdk import (
    ArmSide,
    AxisSelection,
    CartesianPoseMM,
    ControlMode,
    NavigationError,
    NavigationPose,
    Robot,
    ValidationError,
)
from wheelloong_auto_sdk.backend import MockBackend


def make_robot():
    """Create a public Robot and expose its recording MockBackend."""
    backend = MockBackend()
    return Robot.with_backend(backend), backend


def test_dual_move_j_is_normalized_without_ros_types():
    """Normalize dual-arm targets into fixed float tuples without ROS types."""
    robot, backend = make_robot()
    result = robot.arms.move_j(left=[0] * 7, right=[0.1] * 7, speed_rad_s=0.5)
    assert result.ok
    name, values = backend.calls[-1]
    assert name == "move_j"
    assert values["left"] == (0.0,) * 7
    assert values["right"] == (0.1,) * 7


def test_body_omitted_axes_remain_none():
    """Keep omitted head/waist axes unselected and preserve lift direction."""
    robot, backend = make_robot()
    robot.body.move(head_yaw_rad=0.2, waist_lift_mm=-100.0)
    name, values = backend.calls[-1]
    assert name == "move_body"
    assert values["head_pitch"] is None
    assert values["head_yaw"] == 0.2
    assert values["waist_pitch"] is None
    assert values["waist_lift"] == -100.0
    assert values["speed"] == 0.0


def test_system_demo_steps_are_independent_public_calls():
    """Expose mode requests, confirmations and enable requests independently."""
    robot, backend = make_robot()
    axes = AxisSelection(left_arm=True, right_arm=True, head_yaw=True)

    robot.system.set_control_mode(ControlMode.AUTO)
    state = robot.system.wait_for_control_mode(ControlMode.AUTO)
    assert state.control_mode == int(ControlMode.AUTO)

    robot.system.set_enabled(axes)
    state = robot.system.wait_for_enabled(axes)
    assert state.left_arm_enabled
    assert state.right_arm_enabled
    assert state.head_yaw_enabled

    robot.system.set_control_mode(ControlMode.IDLE)
    robot.system.wait_for_control_mode(ControlMode.IDLE)
    robot.system.disable_all()
    state = robot.system.wait_for_all_disabled()
    assert state.control_mode == int(ControlMode.IDLE)
    assert not state.left_arm_enabled
    assert not state.right_arm_enabled
    assert [name for name, _ in backend.calls] == [
        "set_control_mode",
        "set_enabled",
        "set_control_mode",
        "set_enabled",
    ]


def test_arm_defaults_delegate_speed_and_acceleration_to_driver():
    """Pass zero speed and acceleration so the arm driver selects defaults."""
    robot, backend = make_robot()
    robot.arms.move_j(left=[0.0] * 7)
    name, values = backend.calls[-1]
    assert name == "move_j"
    assert values["speed"] == 0.0
    assert values["acceleration"] == 0.0


def test_cartesian_pose_normalizes_quaternion():
    """Normalize explicit quaternions and convert RPY to quaternion form."""
    pose = CartesianPoseMM(1, 2, 3, qw=2).validated()
    assert pose.qw == 1.0
    rpy_pose = CartesianPoseMM.from_rpy(1, 2, 3, 0, 0, math.pi)
    assert rpy_pose.qz == pytest.approx(1.0)


def test_system_state_exposes_current_dual_arm_tcp_poses():
    """通过公共状态对象读取 /system/get_info 的当前双臂 TCP 位姿。"""
    robot, backend = make_robot()
    left = CartesianPoseMM(10.0, 20.0, 30.0)
    right = CartesianPoseMM(-10.0, -20.0, -30.0)
    backend.state = replace(
        backend.state, left_arm_pose=left, right_arm_pose=right
    )
    state = robot.state()
    assert state.left_arm_pose == left
    assert state.right_arm_pose == right


def test_gripper_and_navigation_public_api():
    """Expose gripper state and successful navigation without ROS messages."""
    robot, backend = make_robot()
    robot.gripper.open(ArmSide.DUAL)
    state = robot.state()
    assert state.left_gripper == 1.0
    assert state.right_gripper == 1.0
    handle = robot.navigation.navigate_to(NavigationPose(1.0, 2.0, 0.3))
    assert handle.wait(timeout_sec=1.0).succeeded


def test_navigation_pause_resume_costmaps_and_current_pose():
    """Expose navigation management without leaking ROS message types."""
    robot, backend = make_robot()
    target = NavigationPose(1.0, 2.0, 0.3)
    backend.navigation_pose = NavigationPose(-1.0, 0.5, -0.2)

    assert robot.navigation.clear_local_costmap().ok
    assert robot.navigation.clear_global_costmap().ok
    handle = robot.navigation.navigate_to(target)
    assert not handle.done
    assert handle.pause().status == "CANCELED"
    assert handle.paused
    with pytest.raises(NavigationError, match="paused"):
        handle.wait(timeout_sec=1.0)
    assert handle.resume() is handle
    assert not handle.paused
    pose = robot.navigation.current_pose()
    assert pose == NavigationPose(-1.0, 0.5, -0.2)
    assert handle.wait(timeout_sec=1.0).succeeded

    assert [name for name, _ in backend.calls] == [
        "clear_local_costmap",
        "clear_global_costmap",
        "navigate_to",
        "navigate_to",
        "get_navigation_pose",
    ]


def test_multi_point_resume_reissues_original_sequence():
    """Restart the complete original through route after a soft pause."""
    robot, backend = make_robot()
    poses = [NavigationPose(1.0, 2.0, 0.3), NavigationPose(3.0, 4.0, 0.5)]
    handle = robot.navigation.navigate_through(poses)

    handle.pause()
    handle.resume()

    assert [name for name, _ in backend.calls] == [
        "navigate_through",
        "navigate_through",
    ]
    assert backend.calls[-1][1]["poses"] == tuple(poses)


def test_auto_session_cleans_up_after_exception():
    """Return to IDLE and disable requested axes after a task exception."""
    robot, backend = make_robot()
    axes = AxisSelection(left_arm=True, head_yaw=True)
    with pytest.raises(RuntimeError):
        with robot.auto_session(required_axes=axes):
            assert backend.state.control_mode == 2
            assert backend.state.left_arm_enabled
            raise RuntimeError("task failed")
    assert backend.state.control_mode == 99
    assert not backend.state.left_arm_enabled
    assert not backend.state.head_yaw_enabled
    names = [name for name, _ in backend.calls]
    assert names[:3] == ["set_control_mode", "set_enabled", "arm_hold"]
    assert names[-3:] == ["arm_hold", "set_control_mode", "set_enabled"]


@pytest.mark.parametrize(
    "call",
    [
        lambda robot: robot.arms.move_j(left=[0] * 6),
        lambda robot: robot.arms.move_j(left=[0] * 7, speed_rad_s=5.0),
        lambda robot: robot.system.set_control_mode(1),
        lambda robot: robot.system.set_enabled("all"),
        lambda robot: robot.body.move(),
        lambda robot: robot.body.move(head_pitch_rad=0.0, wait=False, timeout_sec=31.0),
        lambda robot: robot.gripper.set(ArmSide.LEFT, 1.1),
        lambda robot: robot.navigation.navigate_through([]),
        lambda robot: robot.voice.speak("   "),
    ],
)
def test_invalid_inputs_fail_before_backend_call(call):
    """Reject invalid public arguments before any backend side effect."""
    robot, backend = make_robot()
    with pytest.raises(ValidationError):
        call(robot)
    assert backend.calls == []
