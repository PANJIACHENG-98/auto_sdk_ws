"""验证分步运动示例的参数约束和八个独立接口调用。"""

import importlib.util
import math
from pathlib import Path

import pytest

from wheelloong_auto_sdk import Robot, WheelloongSdkError
from wheelloong_auto_sdk.backend import MockBackend


EXAMPLE_PATH = Path(__file__).resolve().parents[1] / "examples" / "separation_demo.py"


def load_example_module():
    """从 examples 目录加载分步运动样例但不执行 main。"""
    spec = importlib.util.spec_from_file_location("separation_demo", EXAMPLE_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_separation_steps_call_each_part_independently():
    """确认八个动作按顺序阻塞调用，并在步骤之间等待七次回车。"""
    example = load_example_module()
    args = example.build_parser().parse_args(["--robot", "shiloong"])
    backend = MockBackend()
    robot = Robot.with_backend(backend)
    prompts = []

    example.run_separation_steps(
        robot, args, input_fn=lambda prompt: prompts.append(prompt)
    )

    assert [name for name, _ in backend.calls] == [
        "move_j",
        "move_j",
        "move_body",
        "move_body",
        "arm_hold",
        "move_p",
        "arm_hold",
        "move_l",
        "set_gripper",
        "set_gripper",
    ]
    left = backend.calls[0][1]
    right = backend.calls[1][1]
    assert left["left"] is not None and left["right"] is None
    assert right["left"] is None and right["right"] is not None
    assert left["left"][1] == pytest.approx(math.radians(77.0))
    assert right["right"][4] == pytest.approx(math.radians(25.0))
    motion_calls = [
        values
        for name, values in backend.calls
        if name in {"move_j", "move_body", "move_p", "move_l"}
    ]
    assert all(values["wait"] for values in motion_calls)
    assert backend.calls[2][1]["head_pitch"] == pytest.approx(
        math.radians(3.0)
    )
    assert backend.calls[2][1]["head_yaw"] == pytest.approx(math.radians(3.0))
    assert backend.calls[2][1]["waist_pitch"] is None
    assert backend.calls[3][1]["head_pitch"] is None
    assert backend.calls[3][1]["waist_lift"] == pytest.approx(-5.0)
    assert backend.calls[5][1]["left"].y_mm == pytest.approx(20.0)
    assert backend.calls[5][1]["right"].y_mm == pytest.approx(-20.0)
    assert backend.calls[7][1]["left"].y_mm == pytest.approx(0.0)
    assert backend.calls[7][1]["right"].y_mm == pytest.approx(0.0)
    assert backend.calls[8][1]["position"] == pytest.approx(0.0)
    assert backend.calls[9][1]["position"] == pytest.approx(1.0)
    assert prompts == [
        "步骤 1/8 已完成，按回车继续...",
        "步骤 2/8 已完成，按回车继续...",
        "步骤 3/8 已完成，按回车继续...",
        "步骤 4/8 已完成，按回车继续...",
        "步骤 5/8 已完成，按回车继续...",
        "步骤 6/8 已完成，按回车继续...",
        "步骤 7/8 已完成，按回车继续...",
    ]


def test_separation_steps_reject_closed_interactive_input():
    """确认无法读取回车时停止后续动作并给出明确错误。"""
    example = load_example_module()
    args = example.build_parser().parse_args(["--robot", "shiloong"])
    backend = MockBackend()
    robot = Robot.with_backend(backend)

    def closed_input(_prompt):
        raise EOFError

    with pytest.raises(WheelloongSdkError, match="interactive input closed"):
        example.run_separation_steps(robot, args, input_fn=closed_input)

    assert [name for name, _ in backend.calls] == ["move_j"]


def test_separation_parser_rejects_out_of_range_joint():
    """确认 Demo 在连接机器人前拒绝超出机械限位的目标。"""
    example = load_example_module()
    parser = example.build_parser()
    args = parser.parse_args(
        [
            "--robot",
            "shiloong",
            "--left-joints-deg",
            "156",
            "0",
            "0",
            "0",
            "0",
            "0",
            "0",
        ]
    )
    with pytest.raises(SystemExit):
        example.validate_arguments(parser, args)
