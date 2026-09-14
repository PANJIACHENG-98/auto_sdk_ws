#!/usr/bin/env python3
"""进入 AUTO 会话、短暂观察状态，然后由 SDK 自动安全退出。

本例不发送 MoveJ、头腰、夹爪、导航或语音目标，只展示 auto_session() 对
AUTO、全部轴使能、双臂 Hold、IDLE 和去使能的统一管理。
"""

import argparse
import sys
import time

from wheelloong_auto_sdk import AxisSelection, Robot, SystemState, WheelloongSdkError
from wheelloong_auto_sdk._cli import non_negative_number, positive_number


def build_parser() -> argparse.ArgumentParser:
    """创建观察时长及状态、服务超时参数。"""
    parser = argparse.ArgumentParser(
        description="Enter and leave auto_session without target motion.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--observe-seconds",
        type=non_negative_number,
        default=2.0,
        help="seconds to remain inside AUTO before normal cleanup",
    )
    parser.add_argument(
        "--state-timeout",
        type=positive_number,
        default=10.0,
        help="seconds to confirm AUTO/IDLE and enable/disable state",
    )
    parser.add_argument(
        "--service-timeout",
        type=positive_number,
        default=3.0,
        help="seconds to wait for each System or Hold service response",
    )
    return parser


def print_state(label: str, state: SystemState) -> None:
    """打印控制模式、六组轴使能值和状态年龄。"""
    enabled = (
        state.left_arm_enabled,
        state.right_arm_enabled,
        state.head_pitch_enabled,
        state.head_yaw_enabled,
        state.waist_pitch_enabled,
        state.waist_lift_enabled,
    )
    print(
        f"[{label}] mode={state.control_mode}, enabled={enabled}, "
        f"age={state.age_sec:.3f}s"
    )


def run_demo(robot: Robot, args: argparse.Namespace) -> None:
    """观察进入、处于和退出 AUTO 会话时的系统状态。"""
    print_state(
        "before",
        robot.system.state(wait_timeout_sec=args.state_timeout),
    )
    with robot.auto_session(
        required_axes=AxisSelection.all(),
        state_timeout_sec=args.state_timeout,
        service_timeout_sec=args.service_timeout,
    ):
        print_state(
            "inside",
            robot.system.state(wait_timeout_sec=args.state_timeout),
        )
        print("no target motion command will be sent")
        if args.observe_seconds > 0.0:
            time.sleep(args.observe_seconds)
    print_state(
        "after",
        robot.system.state(wait_timeout_sec=args.state_timeout),
    )


def main() -> int:
    """创建独立运行时并执行无目标运动的 AUTO 会话示例。"""
    args = build_parser().parse_args()
    try:
        with Robot.standalone(node_name="sdk_auto_session_demo") as robot:
            run_demo(robot, args)
        print("AUTO session completed successfully")
        return 0
    except KeyboardInterrupt:
        print("AUTO session interrupted after safety cleanup", file=sys.stderr)
        return 130
    except WheelloongSdkError as exc:
        print(f"AUTO session failed: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
