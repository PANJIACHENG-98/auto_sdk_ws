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
    RobotStateError,
    ValidationError,
    get_robot_profile,
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


def test_robot_profile_exposes_reusable_shiloong_poses_and_limits():
    """机型配置统一提供工作位、结束位和机械限位。"""
    profile = get_robot_profile("shiloong")
    left, right = profile.work_arm_joints()

    assert math.degrees(left[0]) == pytest.approx(10.0)
    assert math.degrees(right[2]) == pytest.approx(80.0)
    assert profile.validate_arm_degrees(profile.work_left_deg, side="left")
    with pytest.raises(ValidationError, match="J1"):
        profile.validate_arm_degrees([156.0] + [0.0] * 6, side="left")


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
    assert state.left_gripper == 0.0
    assert state.right_gripper == 0.0
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


@pytest.mark.parametrize(
    ("poses", "expected"),
    (
        ([NavigationPose(1.0, 2.0, 0.3)], "navigate_to"),
        (
            [NavigationPose(1.0, 2.0, 0.3), NavigationPose(3.0, 4.0, 0.5)],
            "navigate_through",
        ),
    ),
)
def test_navigation_sequence_selects_single_or_multi_action(poses, expected):
    """统一 navigate() 根据目标数量选择单点或多点 Action。"""
    robot, backend = make_robot()

    robot.navigation.navigate(poses)

    assert backend.calls[-1][0] == expected


def test_system_require_ready_checks_mode_axes_and_errors():
    """统一就绪检查同时覆盖 AUTO、使能和活动错误。"""
    robot, backend = make_robot()
    axes = AxisSelection(left_arm=True)
    backend.state = replace(
        backend.state,
        control_mode=int(ControlMode.AUTO),
        left_arm_enabled=True,
    )
    assert robot.system.require_ready(axes).left_arm_enabled

    backend.state = replace(backend.state, errors=("fault",))
    with pytest.raises(RobotStateError, match="fault"):
        robot.system.require_ready(axes)


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


def test_auto_session_can_preserve_an_initial_auto_mode():
    """无轴导航会话可在退出后保留进入前已有的 AUTO。"""
    robot, backend = make_robot()
    backend.state = replace(
        backend.state,
        control_mode=int(ControlMode.AUTO),
    )

    with robot.auto_session(
        required_axes=AxisSelection.none(),
        restore_initial_mode=True,
    ):
        pass

    mode_requests = [
        values["mode"]
        for name, values in backend.calls
        if name == "set_control_mode"
    ]
    assert mode_requests == [int(ControlMode.AUTO)]


def test_auto_session_holds_arms_after_keyboard_interrupt():
    """Ctrl+C 后先 Hold 双臂，再切换 IDLE 并去使能。"""
    robot, backend = make_robot()
    axes = AxisSelection(left_arm=True, right_arm=True)

    with pytest.raises(KeyboardInterrupt):
        with robot.auto_session(required_axes=axes):
            raise KeyboardInterrupt

    names = [name for name, _ in backend.calls]
    cleanup_start = len(names) - 3
    assert names[cleanup_start:] == ["arm_hold", "set_control_mode", "set_enabled"]
    assert backend.state.control_mode == 99
    assert not backend.state.left_arm_enabled
    assert not backend.state.right_arm_enabled


def test_auto_session_holds_arms_when_interrupted_during_entry(monkeypatch):
    """进入 AUTO 过程中收到 Ctrl+C 也执行 Hold 和退出清理。"""
    robot, backend = make_robot()
    original_set_control_mode = backend.set_control_mode

    def interrupt_auto_request(mode, timeout_sec):
        if int(mode) == 2:
            raise KeyboardInterrupt
        return original_set_control_mode(mode, timeout_sec)

    monkeypatch.setattr(backend, "set_control_mode", interrupt_auto_request)

    with pytest.raises(KeyboardInterrupt):
        with robot.auto_session(
            required_axes=AxisSelection(left_arm=True, right_arm=True)
        ):
            pytest.fail("AUTO session must not enter after Ctrl+C")

    names = [name for name, _ in backend.calls]
    assert names == ["arm_hold", "set_control_mode"]
    assert backend.state.control_mode == 99


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
        lambda robot: robot.navigation.navigate([]),
        lambda robot: robot.navigation.navigate_through([]),
        lambda robot: robot.navigation.navigate_waypoints([True]),
        lambda robot: robot.voice.speak("   "),
    ],
)
def test_invalid_inputs_fail_before_backend_call(call):
    """Reject invalid public arguments before any backend side effect."""
    robot, backend = make_robot()
    with pytest.raises(ValidationError):
        call(robot)
    assert backend.calls == []
