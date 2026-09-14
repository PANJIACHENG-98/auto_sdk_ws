"""验证移动操作 Demo 的导航、八步动作、播报和下电顺序。"""

from pathlib import Path

import pytest

from examples import mobile_manipulation_demo as example
from wheelloong_auto_sdk import CommandTimeoutError, ControlMode, Robot
from wheelloong_auto_sdk.backend import MockBackend


WAYPOINT_PATH = Path(__file__).resolve().parents[1] / "waypoint" / "waypoints.txt"


def parse_default_arguments():
    """创建指向测试点位文件的默认任务参数。"""
    parser = example.build_parser()
    args = parser.parse_args(
        ["--robot", "shiloong", "--file", str(WAYPOINT_PATH)]
    )
    example.validate_arguments(parser, args)
    return args


def test_complete_mobile_manipulation_call_order():
    """确认点位 1/2 之间各执行一轮八步动作并最后播报。"""
    args = parse_default_arguments()
    backend = MockBackend()
    robot = Robot.with_backend(backend)
    delays = []

    example.run_demo(robot, args, sleep_fn=delays.append)

    names = [name for name, _ in backend.calls]
    assert names == [
        "set_control_mode",
        "set_enabled",
        "arm_hold",
        "clear_local_costmap",
        "clear_global_costmap",
        "navigate_to",
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
        "clear_local_costmap",
        "clear_global_costmap",
        "navigate_to",
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
        "speak",
        "arm_hold",
        "set_control_mode",
        "set_enabled",
    ]
    navigation_calls = [
        values for name, values in backend.calls if name == "navigate_to"
    ]
    assert navigation_calls[0]["pose"].x_m == pytest.approx(-2.592143711)
    assert navigation_calls[1]["pose"].x_m == pytest.approx(-3.823100147)
    assert delays == [example.STEP_INTERVAL_SEC] * 18
    assert backend.calls[-4] == (
        "speak",
        {"text": example.DEFAULT_SPEECH_TEXT, "wait": True},
    )
    assert backend.state.control_mode == int(ControlMode.IDLE)
    assert not backend.state.left_arm_enabled
    assert not backend.state.right_arm_enabled


def test_mobile_manipulation_requires_both_waypoints(tmp_path):
    """确认在连接真机前拒绝缺少点位 2 的文件。"""
    waypoint_file = tmp_path / "waypoints.txt"
    waypoint_file.write_text(
        "# id,x_m,y_m,yaw_rad,frame_id\n1,0,0,0,map\n",
        encoding="utf-8",
    )
    parser = example.build_parser()
    args = parser.parse_args(
        ["--robot", "shiloong", "--file", str(waypoint_file)]
    )

    with pytest.raises(SystemExit) as error:
        example.validate_arguments(parser, args)

    assert error.value.code == 2


def test_speech_failure_still_holds_and_powers_down(monkeypatch):
    """确认最后的语音超时也会触发 Hold、IDLE 和全部去使能。"""
    args = parse_default_arguments()
    backend = MockBackend()
    robot = Robot.with_backend(backend)

    def fail_speech(*_args, **_kwargs):
        """模拟语音服务等待超时。"""
        raise CommandTimeoutError("simulated speech timeout")

    monkeypatch.setattr(robot.voice, "speak", fail_speech)
    with pytest.raises(CommandTimeoutError, match="speech timeout"):
        example.run_demo(robot, args, sleep_fn=lambda _seconds: None)

    assert [name for name, _ in backend.calls][-3:] == [
        "arm_hold",
        "set_control_mode",
        "set_enabled",
    ]
    assert backend.state.control_mode == int(ControlMode.IDLE)
    assert not backend.state.left_arm_enabled
    assert not backend.state.right_arm_enabled
