#!/usr/bin/env python3
"""分步验证侍龙 L4 各运动接口。

本例会真实控制机器人。八个步骤均阻塞等待当前动作完成，用户按回车后才继续；
用于逐项观察 MoveJ、头腰、MoveP、MoveL 和夹爪接口，不控制底盘。
"""

import argparse
import math
import sys
from typing import Callable

from wheelloong_auto_sdk import (
    ArmSide,
    AxisSelection,
    CartesianPoseMM,
    Robot,
    SHILOONG_PROFILE,
    ValidationError,
    WheelloongSdkError,
)
from wheelloong_auto_sdk._cli import (
    finite_number,
    non_negative_number,
    positive_number,
)


def build_parser() -> argparse.ArgumentParser:
    """创建侍龙机型、四组目标、速度和超时参数解析器。"""
    parser = argparse.ArgumentParser(
        description=(
            "Move the Shiloong left arm, right arm, head, and waist "
            "one step at a time."
        ),
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--robot",
        choices=("shiloong",),
        required=True,
        help="robot model safety acknowledgement",
    )
    parser.add_argument(
        "--left-joints-deg",
        nargs=7,
        type=finite_number,
        default=SHILOONG_PROFILE.work_left_deg,
        metavar=("J1", "J2", "J3", "J4", "J5", "J6", "J7"),
        help="left-arm absolute MoveJ target in degrees",
    )
    parser.add_argument(
        "--right-joints-deg",
        nargs=7,
        type=finite_number,
        default=SHILOONG_PROFILE.work_right_deg,
        metavar=("J1", "J2", "J3", "J4", "J5", "J6", "J7"),
        help="right-arm absolute MoveJ target in degrees",
    )
    parser.add_argument("--head-pitch-deg", type=finite_number, default=3.0)
    parser.add_argument("--head-yaw-deg", type=finite_number, default=3.0)
    parser.add_argument("--waist-pitch-deg", type=finite_number, default=3.0)
    parser.add_argument("--waist-lift-mm", type=finite_number, default=-5.0)
    parser.add_argument(
        "--arm-velocity",
        type=non_negative_number,
        default=0.0,
        help="MoveJ velocity in rad/s; 0 uses the driver default",
    )
    parser.add_argument(
        "--arm-acceleration",
        type=non_negative_number,
        default=0.0,
        help="MoveJ acceleration in rad/s^2; 0 uses the driver default",
    )
    parser.add_argument(
        "--body-speed-percent",
        type=non_negative_number,
        default=0.0,
        help="head/waist speed percentage; 0 uses the driver default",
    )
    parser.add_argument(
        "--move-p-speed", type=non_negative_number, default=0.2
    )
    parser.add_argument(
        "--move-l-speed", type=non_negative_number, default=10.0
    )
    parser.add_argument("--state-timeout", type=positive_number, default=10.0)
    parser.add_argument("--service-timeout", type=positive_number, default=3.0)
    parser.add_argument("--motion-timeout", type=positive_number, default=30.0)
    return parser


def validate_arguments(
    parser: argparse.ArgumentParser, args: argparse.Namespace
) -> None:
    """校验所有目标都处于侍龙 L4 文档规定的机械范围内。"""
    try:
        SHILOONG_PROFILE.validate_arm_degrees(
            args.left_joints_deg, side="left"
        )
        SHILOONG_PROFILE.validate_arm_degrees(
            args.right_joints_deg, side="right"
        )
    except ValidationError as exc:
        parser.error(str(exc))
    ranges = (
        (
            "--head-pitch-deg",
            args.head_pitch_deg,
            *SHILOONG_PROFILE.head_pitch_limits_deg,
        ),
        (
            "--head-yaw-deg",
            args.head_yaw_deg,
            *SHILOONG_PROFILE.head_yaw_limits_deg,
        ),
        (
            "--waist-pitch-deg",
            args.waist_pitch_deg,
            *SHILOONG_PROFILE.waist_pitch_limits_deg,
        ),
        (
            "--waist-lift-mm",
            args.waist_lift_mm,
            *SHILOONG_PROFILE.waist_lift_limits_mm,
        ),
        ("--body-speed-percent", args.body_speed_percent, 0.0, 100.0),
    )
    for name, value, minimum, maximum in ranges:
        if not minimum <= value <= maximum:
            parser.error(f"{name} must be in [{minimum}, {maximum}]")
    if args.arm_velocity > 3.14:
        parser.error("--arm-velocity must not exceed 3.14 rad/s")
    if args.motion_timeout > 30.0:
        parser.error("--motion-timeout must not exceed 30 seconds")


def wait_for_enter(completed_step: int, input_fn: Callable[[str], str]) -> None:
    """等待用户确认后再执行下一个部件运动。"""
    try:
        input_fn(f"步骤 {completed_step}/8 已完成，按回车继续...")
    except EOFError as exc:
        raise WheelloongSdkError(
            "interactive input closed before the next motion"
        ) from exc


def run_separation_steps(
    robot: Robot,
    args: argparse.Namespace,
    input_fn: Callable[[str], str] = input,
) -> None:
    """依次阻塞执行八个运动，并在相邻步骤之间等待用户按回车。"""
    print("step 1/8: move left arm with MoveJ")
    robot.arms.move_j(
        left=tuple(math.radians(value) for value in args.left_joints_deg),
        speed_rad_s=args.arm_velocity,
        acceleration_rad_s2=args.arm_acceleration,
        wait=True,
        timeout_sec=args.motion_timeout,
    )
    wait_for_enter(1, input_fn)

    print("step 2/8: move right arm with MoveJ")
    robot.arms.move_j(
        right=tuple(math.radians(value) for value in args.right_joints_deg),
        speed_rad_s=args.arm_velocity,
        acceleration_rad_s2=args.arm_acceleration,
        wait=True,
        timeout_sec=args.motion_timeout,
    )
    wait_for_enter(2, input_fn)

    print("step 3/8: move head")
    robot.body.move_head(
        pitch_rad=math.radians(args.head_pitch_deg),
        yaw_rad=math.radians(args.head_yaw_deg),
        speed_percent=args.body_speed_percent,
        wait=True,
        timeout_sec=args.motion_timeout,
    )
    wait_for_enter(3, input_fn)

    print("step 4/8: move waist")
    robot.body.move_waist(
        pitch_rad=math.radians(args.waist_pitch_deg),
        lift_mm=args.waist_lift_mm,
        speed_percent=args.body_speed_percent,
        wait=True,
        timeout_sec=args.motion_timeout,
    )
    wait_for_enter(4, input_fn)

    print("step 5/8: move both arms with MoveP local offset")
    robot.arms.move_p_offset(
        left=CartesianPoseMM.from_rpy_degrees(
            0.0, 20.0, 30.0, 0.0, 0.0, 0.0
        ),
        right=CartesianPoseMM.from_rpy_degrees(
            0.0, -20.0, 30.0, 0.0, 0.0, 0.0
        ),
        speed_rad_s=args.move_p_speed,
        wait=True,
        timeout_sec=args.motion_timeout,
        hold_timeout_sec=args.service_timeout,
        state_timeout_sec=args.state_timeout,
    )
    wait_for_enter(5, input_fn)

    print("step 6/8: move both arms with MoveL local offset")
    robot.arms.move_l_offset(
        left=CartesianPoseMM.from_rpy_degrees(
            0.0, -20.0, -30.0, 0.0, 0.0, 0.0
        ),
        right=CartesianPoseMM.from_rpy_degrees(
            0.0, 20.0, -30.0, 0.0, 0.0, 0.0
        ),
        speed_mm_s=args.move_l_speed,
        wait=True,
        timeout_sec=args.motion_timeout,
        hold_timeout_sec=args.service_timeout,
        state_timeout_sec=args.state_timeout,
    )
    wait_for_enter(6, input_fn)

    print("step 7/8: open both grippers")
    robot.gripper.open(
        ArmSide.DUAL,
        wait=True,
        timeout_sec=args.motion_timeout,
    )
    wait_for_enter(7, input_fn)

    print("step 8/8: close both grippers")
    robot.gripper.close(
        ArmSide.DUAL,
        wait=True,
        timeout_sec=args.motion_timeout,
    )


def main() -> int:
    """创建独立运行时，在 AUTO 会话中执行八步运动并安全清理。"""
    parser = build_parser()
    args = parser.parse_args()
    validate_arguments(parser, args)
    try:
        with Robot.standalone(node_name="sdk_separation_demo") as robot:
            with robot.auto_session(
                required_axes=AxisSelection.all(),
                state_timeout_sec=args.state_timeout,
                service_timeout_sec=args.service_timeout,
            ):
                try:
                    run_separation_steps(robot, args)
                except KeyboardInterrupt:
                    print(
                        "收到 Ctrl+C，正在请求双臂 Hold 并清理 AUTO 会话...",
                        file=sys.stderr,
                    )
                    raise
        print("separation demo completed")
        return 0
    except KeyboardInterrupt:
        print("separation demo interrupted", file=sys.stderr)
        return 130
    except WheelloongSdkError as exc:
        print(f"separation demo failed: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
