"""Validate that the hardware oscillation example tracks the official Demo."""

import importlib.util
from dataclasses import replace
from pathlib import Path

from wheelloong_auto_sdk import ArmSide, CartesianPoseMM, Robot
from wheelloong_auto_sdk.backend import MockBackend


EXAMPLE_PATH = (
    Path(__file__).resolve().parents[1]
    / "examples"
    / "auto_oscillation_demo.py"
)


def load_example_module():
    """从 examples 目录加载硬件样例而不执行其 main。"""
    spec = importlib.util.spec_from_file_location(
        "auto_oscillation_demo", EXAMPLE_PATH
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_sample_pose_matches_official_demo_request_defaults():
    """确认样例按官方默认值依次下发双臂、头腰和双夹爪。"""
    example = load_example_module()
    args = example.build_parser().parse_args(["--robot", "shiloong"])
    backend = MockBackend()
    robot = Robot.with_backend(backend)

    example.command_test_pose(
        robot=robot,
        args=args,
        left=[0.1] * 7,
        right=[0.2] * 7,
        body_targets=[0.03, 0.04, 0.05, -5.0],
        gripper_position=0.4,
        blocking=False,
    )

    assert [name for name, _ in backend.calls] == [
        "move_j",
        "move_body",
        "set_gripper",
    ]
    arm = backend.calls[0][1]
    assert arm["speed"] == 0.0
    assert arm["acceleration"] == 0.0
    assert not arm["wait"]
    body = backend.calls[1][1]
    assert body["head_pitch"] == 0.03
    assert body["head_yaw"] == 0.04
    assert body["waist_pitch"] == 0.05
    assert body["waist_lift"] == -5.0
    assert body["speed"] == 0.0
    gripper = backend.calls[2][1]
    assert gripper["side"] == int(ArmSide.DUAL)
    assert gripper["position"] == 0.4


def test_parser_defaults_match_installed_official_demo():
    """Lock the installed Demo's current trajectory and timeout defaults."""
    example = load_example_module()
    args = example.build_parser().parse_args(["--robot", "shiloong"])
    assert args.duration == 20.0
    assert args.period == 4.0
    assert args.rate == 5.0
    assert args.amplitude_deg == 3.0
    assert args.lumbar_lift_amplitude_mm == 100.0
    assert args.control_gripper is True
    assert args.work_hold == 2.0
    assert args.state_timeout == 10.0
    assert args.service_timeout == 3.0
    assert args.motion_timeout == 60.0
    assert args.arm_velocity == 0.0
    assert args.arm_acceleration == 0.0


def test_shiloong_final_pose_matches_official_demo_degrees():
    """确认侍龙结束姿态与官方 Demo 常量一致。"""
    example = load_example_module()
    left, right = example.return_arm_targets("shiloong")
    assert tuple(round(example.math.degrees(value), 6) for value in left) == (
        0.0,
        85.0,
        -2.0,
        3.0,
        -25.0,
        0.0,
        0.0,
    )
    assert tuple(round(example.math.degrees(value), 6) for value in right) == (
        0.0,
        85.0,
        -2.0,
        3.0,
        25.0,
        0.0,
        0.0,
    )


def test_shiloong_oscillation_is_centered_on_work_pose():
    """确认正弦偏移逐轴叠加到新增的左右臂工作姿态。"""
    example = load_example_module()
    offset_rad = example.math.radians(3.0)
    left, right = example.oscillating_arm_targets("shiloong", offset_rad)

    left_deg = tuple(round(example.math.degrees(value), 6) for value in left)
    right_deg = tuple(round(example.math.degrees(value), 6) for value in right)
    assert left_deg == (13.0, 80.0, -77.0, 23.0, -22.0, 13.0, 13.0)
    assert right_deg == (-7.0, 80.0, 83.0, 23.0, 28.0, -7.0, 13.0)


def test_move_p_then_move_l_offsets_use_each_arm_pose_and_shared_speed(
    monkeypatch,
):
    """确认两个 offset 接口使用左右六维偏移和共同速度。"""
    example = load_example_module()
    args = example.build_parser().parse_args(["--robot", "shiloong"])
    backend = MockBackend()
    left_current = CartesianPoseMM(100.0, 200.0, -300.0)
    right_current = CartesianPoseMM(-100.0, -200.0, -310.0)
    backend.state = replace(
        backend.state,
        left_arm_pose=left_current,
        right_arm_pose=right_current,
    )
    robot = Robot.with_backend(backend)
    monkeypatch.setattr(example.time, "sleep", lambda _seconds: None)

    example.command_move_p_offset(
        robot,
        args,
        left_offset_6d=(10.0, 0.0, 30.0, 0.0, 0.0, 5.0),
        right_offset_6d=(-10.0, 0.0, 40.0, 0.0, 0.0, -5.0),
        speed_rad_s=0.3,
    )
    example.command_move_l_offset(
        robot,
        args,
        left_offset_6d=(0.0, 20.0, 30.0, 1.0, 0.0, 0.0),
        right_offset_6d=(0.0, -20.0, 40.0, -1.0, 0.0, 0.0),
    )

    assert [name for name, _ in backend.calls] == [
        "arm_hold",
        "move_p",
        "arm_hold",
        "move_l",
    ]
    point = backend.calls[1][1]
    assert point["left"].x_mm == left_current.x_mm + 10.0
    assert point["right"].x_mm == right_current.x_mm - 10.0
    assert point["left"].z_mm == left_current.z_mm + 30.0
    assert point["right"].z_mm == right_current.z_mm + 40.0
    assert point["left"].qz != left_current.qz
    assert point["right"].qz != right_current.qz
    assert point["speed"] == 0.3
    assert point["wait"] is True
    line = backend.calls[3][1]
    assert int(line["mode"]) == 0
    assert line["left"].y_mm == point["left"].y_mm + 20.0
    assert line["right"].y_mm == point["right"].y_mm - 20.0
    assert line["left"].z_mm == point["left"].z_mm + 30.0
    assert line["right"].z_mm == point["right"].z_mm + 40.0
    assert line["left"].qx != point["left"].qx
    assert line["right"].qx != point["right"].qx
    assert line["speed"] == 10.0
    assert line["wait"] is True


def test_absolute_cartesian_commands_accept_only_dual_six_d_poses():
    """确认绝对接口直接转换左右六维位姿并使用共同默认速度。"""
    example = load_example_module()
    args = example.build_parser().parse_args(["--robot", "shiloong"])
    backend = MockBackend()
    robot = Robot.with_backend(backend)

    example.command_move_p_absolute(
        robot,
        args,
        left_pose_6d=(100.0, 200.0, -300.0, 0.0, 0.0, 10.0),
        right_pose_6d=(-100.0, -200.0, -310.0, 0.0, 0.0, -10.0),
    )
    example.command_move_l_absolute(
        robot,
        args,
        left_pose_6d=(110.0, 210.0, -290.0, 1.0, 2.0, 3.0),
        right_pose_6d=(-110.0, -210.0, -300.0, -1.0, -2.0, -3.0),
    )

    assert [name for name, _ in backend.calls] == ["move_p", "move_l"]
    point = backend.calls[0][1]
    assert point["left"].x_mm == 100.0
    assert point["right"].z_mm == -310.0
    assert point["speed"] == 0.2
    line = backend.calls[1][1]
    assert int(line["mode"]) == 0
    assert line["left"].y_mm == 210.0
    assert line["right"].z_mm == -300.0
    assert line["speed"] == 10.0


def test_full_demo_uses_explicit_system_and_cartesian_interfaces(monkeypatch):
    """确认完整样例依次执行系统、MoveP、MoveL 和结束姿态。"""
    example = load_example_module()
    args = example.build_parser().parse_args(["--robot", "shiloong"])
    backend = MockBackend()
    robot = Robot.with_backend(backend)
    monkeypatch.setattr(example.time, "sleep", lambda _seconds: None)
    monkeypatch.setattr(
        example,
        "run_oscillation_samples",
        lambda _robot, _args, _axes: None,
    )

    example.run_demo(robot, args)

    names = [name for name, _ in backend.calls]
    assert names[:3] == ["set_control_mode", "set_enabled", "arm_hold"]
    assert names[-2:] == ["set_control_mode", "set_enabled"]
    assert backend.calls[0][1]["mode"] == 2
    work_move = backend.calls[3][1]
    expected_left, expected_right = example.work_arm_targets("shiloong")
    assert work_move["left"] == tuple(expected_left)
    assert work_move["right"] == tuple(expected_right)
    move_p_index = names.index("move_p")
    move_l_index = names.index("move_l")
    final_move_j_index = len(names) - 1 - names[::-1].index("move_j")
    assert move_p_index < move_l_index < final_move_j_index
    assert names[move_p_index + 1] == "arm_hold"
    assert names[move_l_index + 1] == "arm_hold"
    assert names[move_l_index + 2] == "move_j"
    assert backend.calls[-2][1]["mode"] == 99
    assert backend.calls[-1][1]["axes"] == example.AxisSelection.none()
