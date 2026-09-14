"""Validate that the session example delegates lifecycle work to the SDK."""

import importlib.util
from pathlib import Path

import pytest

from wheelloong_auto_sdk import ControlMode, Robot
from wheelloong_auto_sdk.backend import MockBackend


EXAMPLE_PATH = Path(__file__).resolve().parents[1] / "examples" / "auto_session_demo.py"


def load_example_module():
    """从 examples 目录加载纯 AUTO 会话样例而不执行 main。"""
    spec = importlib.util.spec_from_file_location("auto_session_demo", EXAMPLE_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_auto_session_example_is_motion_free(monkeypatch):
    """确认示例仅执行生命周期命令，不下发任何目标运动。"""
    example = load_example_module()
    args = example.build_parser().parse_args(["--observe-seconds", "0"])
    backend = MockBackend()
    robot = Robot.with_backend(backend)

    example.run_demo(robot, args)

    names = [name for name, _ in backend.calls]
    assert names == [
        "set_control_mode",
        "set_enabled",
        "arm_hold",
        "arm_hold",
        "set_control_mode",
        "set_enabled",
    ]
    forbidden = {"move_j", "move_l", "move_p", "move_body", "set_gripper"}
    assert forbidden.isdisjoint(names)
    assert backend.state.control_mode == int(ControlMode.IDLE)
    assert not backend.state.left_arm_enabled
    assert not backend.state.right_arm_enabled


def test_auto_session_example_cleans_up_keyboard_interrupt(monkeypatch):
    """观察阶段按 Ctrl+C 时仍 Hold、IDLE 并去使能。"""
    example = load_example_module()
    args = example.build_parser().parse_args(["--observe-seconds", "1"])
    backend = MockBackend()
    robot = Robot.with_backend(backend)

    def interrupt(_seconds):
        raise KeyboardInterrupt

    monkeypatch.setattr(example.time, "sleep", interrupt)
    with pytest.raises(KeyboardInterrupt):
        example.run_demo(robot, args)

    names = [name for name, _ in backend.calls]
    assert names[-3:] == ["arm_hold", "set_control_mode", "set_enabled"]
    assert backend.state.control_mode == int(ControlMode.IDLE)
