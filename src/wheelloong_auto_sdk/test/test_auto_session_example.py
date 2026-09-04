"""Validate that the session-only example sends no target-motion command."""

import importlib.util
from pathlib import Path

import pytest

from wheelloong_auto_sdk import ControlMode, Robot, RobotCommandError
from wheelloong_auto_sdk.backend import MockBackend


EXAMPLE_PATH = Path(__file__).resolve().parents[1] / "examples" / "auto_session_demo.py"


def load_example_module():
    """从 examples 目录加载纯 AUTO 会话样例而不执行 main。"""
    spec = importlib.util.spec_from_file_location("auto_session_demo", EXAMPLE_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_two_stage_functions_are_independent_and_motion_free():
    """确认两个函数可分开调用，且只执行模式、使能和 Hold。"""
    example = load_example_module()
    args = example.build_parser().parse_args([])
    backend = MockBackend()
    robot = Robot.with_backend(backend)

    example.enter_auto_stage(robot, args)
    assert [name for name, _ in backend.calls] == [
        "set_control_mode",
        "set_enabled",
        "arm_hold",
    ]
    assert backend.state.control_mode == int(ControlMode.AUTO)
    assert backend.state.left_arm_enabled
    assert backend.state.right_arm_enabled

    example.exit_auto_stage(robot, args)

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


def test_run_demo_executes_stage_two_after_stage_one_error(monkeypatch):
    """确认阶段一异常时 run_demo 仍调用独立阶段二进行清理。"""
    example = load_example_module()
    args = example.build_parser().parse_args([])
    backend = MockBackend()
    robot = Robot.with_backend(backend)
    stages = []

    def fail_stage_one(_robot, _args):
        """记录阶段一并模拟其中途失败。"""
        stages.append("enter")
        raise RuntimeError("stage one failed")

    def record_stage_two(_robot, _args):
        """记录异常路径仍调用了阶段二。"""
        stages.append("exit")

    monkeypatch.setattr(example, "enter_auto_stage", fail_stage_one)
    monkeypatch.setattr(example, "exit_auto_stage", record_stage_two)
    with pytest.raises(RuntimeError, match="stage one failed"):
        example.run_demo(robot, args)
    assert stages == ["enter", "exit"]


def test_exit_skips_hold_when_arms_are_not_enabled(capsys):
    """确认中断发生在使能前时不会调用必然失败的 Hold。"""
    example = load_example_module()
    args = example.build_parser().parse_args([])
    backend = MockBackend()
    robot = Robot.with_backend(backend)

    example.exit_auto_stage(robot, args)

    names = [name for name, _ in backend.calls]
    assert "arm_hold" not in names
    assert names == ["set_control_mode", "set_enabled"]
    assert "[跳过 Hold]" in capsys.readouterr().out


def test_step_error_identifies_robot_rejection_and_error_code(monkeypatch):
    """确认底层拒绝命令时输出阶段、步骤、错误码和原始信息。"""
    example = load_example_module()
    args = example.build_parser().parse_args([])
    robot = Robot.with_backend(MockBackend())

    def reject_initial_state(**_kwargs):
        """模拟底层在读取初始状态时拒绝操作。"""
        raise RobotCommandError("read state", 7, "模拟拒绝原因")

    monkeypatch.setattr(robot.system, "state", reject_initial_state)
    with pytest.raises(example.DemoStepError) as caught:
        example.enter_auto_stage(robot, args)

    message = str(caught.value)
    assert "阶段一 / 读取初始系统状态" in message
    assert "机器人拒绝命令" in message
    assert "error_code=7" in message
    assert "模拟拒绝原因" in message
