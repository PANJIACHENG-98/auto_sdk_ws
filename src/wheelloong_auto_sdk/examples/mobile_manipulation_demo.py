#!/usr/bin/env python3
"""按明确顺序执行导航、本体操作、语音播报和下电。

本 Demo 不调用其他 Demo 的任何函数，所有 SDK 调用均按真实执行
顺序平铺在 run_demo() 中。本体动作目标与 separation_demo 现有的
八步目标一致，相邻任务动作之间等待 500 ms。
"""

import argparse
import math
from pathlib import Path
import sys
import time
from typing import Callable

from wheelloong_auto_sdk import (
    ArmSide,
    AxisSelection,
    CartesianPoseMM,
    ControlMode,
    Robot,
    RobotStateError,
    SHILOONG_PROFILE,
    ValidationError,
    WheelloongSdkError,
    default_waypoint_path,
    load_waypoints,
)
from wheelloong_auto_sdk._cli import (
    finite_number,
    non_negative_number,
    positive_number,
)


STEP_INTERVAL_SEC = 0.5
DEFAULT_SPEECH_TEXT = "移动抓取任务已完成"


def build_parser() -> argparse.ArgumentParser:
    """创建导航、本体目标、语音和超时参数解析器。"""
    parser = argparse.ArgumentParser(
        description=(
            "Navigate to waypoints 1 and 2, run eight body steps at each "
            "point, announce completion, then enter IDLE and disable axes."
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
        "--move-p-speed",
        type=non_negative_number,
        default=0.2,
    )
    parser.add_argument(
        "--move-l-speed",
        type=non_negative_number,
        default=10.0,
    )
    parser.add_argument(
        "--file",
        type=Path,
        default=default_waypoint_path(),
        help="waypoint file containing ids 1 and 2",
    )
    parser.add_argument(
        "--navigation-timeout",
        type=positive_number,
        default=300.0,
        help="maximum seconds allowed for each navigation",
    )
    parser.add_argument(
        "--server-timeout",
        type=positive_number,
        default=5.0,
        help="Nav2 server and costmap service timeout",
    )
    parser.add_argument(
        "--speech-text",
        default=DEFAULT_SPEECH_TEXT,
        help="completion text passed to voice.speak",
    )
    parser.add_argument(
        "--speech-timeout",
        type=positive_number,
        default=30.0,
        help="speech synthesis and playback timeout",
    )
    parser.add_argument(
        "--speech-call-timeout",
        type=positive_number,
        default=35.0,
        help="maximum service response wait for voice.speak",
    )
    parser.add_argument("--state-timeout", type=positive_number, default=10.0)
    parser.add_argument("--service-timeout", type=positive_number, default=3.0)
    parser.add_argument("--motion-timeout", type=positive_number, default=30.0)
    return parser


def validate_arguments(
    parser: argparse.ArgumentParser,
    args: argparse.Namespace,
) -> None:
    """在连接机器人前校验本体目标、导航点和播报文本。"""
    try:
        SHILOONG_PROFILE.validate_arm_degrees(
            args.left_joints_deg,
            side="left",
        )
        SHILOONG_PROFILE.validate_arm_degrees(
            args.right_joints_deg,
            side="right",
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
    if not str(args.speech_text).strip():
        parser.error("--speech-text must not be empty")

    try:
        waypoints = load_waypoints(args.file)
    except (OSError, ValidationError) as exc:
        parser.error(str(exc))
    missing = [
        waypoint_id
        for waypoint_id in (1, 2)
        if waypoint_id not in waypoints
    ]
    if missing:
        parser.error(
            "waypoint file must contain ids 1 and 2; missing: "
            + ", ".join(str(item) for item in missing)
        )


def run_demo(
    robot: Robot,
    args: argparse.Namespace,
    sleep_fn: Callable[[float], None] = time.sleep,
) -> None:
    """平铺执行从切换 AUTO、上使能到 IDLE、去使能的全流程。"""
    all_axes = AxisSelection.all()
    auto_requested = False
    enable_requested = False
    task_failed = False

    try:
        print("stage 1: check initial System state")
        initial_state = robot.system.state(
            max_age_sec=0.5,
            wait_timeout_sec=args.state_timeout,
        )
        if initial_state.errors:
            raise RobotStateError(
                "robot reports active errors before AUTO entry: "
                + "; ".join(initial_state.errors)
            )

        print("stage 2: switch System to AUTO")
        auto_requested = True
        robot.system.set_control_mode(
            ControlMode.AUTO,
            timeout_sec=args.service_timeout,
        )
        robot.system.wait_for_control_mode(
            ControlMode.AUTO,
            timeout_sec=args.state_timeout,
        )

        print("stage 3: enable both arms, head, and waist")
        enable_requested = True
        robot.system.enable_all(timeout_sec=args.service_timeout)
        robot.system.wait_for_enabled(
            all_axes,
            timeout_sec=args.state_timeout,
        )

        print("stage 4: Hold both arms before navigation")
        robot.arms.hold(timeout_sec=args.service_timeout)

        print("stage 5: navigate to waypoint 1")
        robot.navigation.clear_local_costmap(
            timeout_sec=args.server_timeout
        )
        robot.navigation.clear_global_costmap(
            timeout_sec=args.server_timeout
        )
        waypoint_1_navigation = robot.navigation.navigate_waypoints(
            (1,),
            path=args.file,
            server_timeout_sec=args.server_timeout,
        )
        waypoint_1_result = waypoint_1_navigation.wait(
            timeout_sec=args.navigation_timeout,
            cancel_on_timeout=True,
            cancel_timeout_sec=args.server_timeout,
        )
        print(f"waypoint 1 navigation finished: {waypoint_1_result.status}")
        sleep_fn(STEP_INTERVAL_SEC)

        print("waypoint 1, step 1/8: move left arm with MoveJ")
        robot.arms.move_j(
            left=tuple(
                math.radians(value) for value in args.left_joints_deg
            ),
            speed_rad_s=args.arm_velocity,
            acceleration_rad_s2=args.arm_acceleration,
            wait=True,
            timeout_sec=args.motion_timeout,
        )
        sleep_fn(STEP_INTERVAL_SEC)

        print("waypoint 1, step 2/8: move right arm with MoveJ")
        robot.arms.move_j(
            right=tuple(
                math.radians(value) for value in args.right_joints_deg
            ),
            speed_rad_s=args.arm_velocity,
            acceleration_rad_s2=args.arm_acceleration,
            wait=True,
            timeout_sec=args.motion_timeout,
        )
        sleep_fn(STEP_INTERVAL_SEC)

        print("waypoint 1, step 3/8: move head")
        robot.body.move_head(
            pitch_rad=math.radians(args.head_pitch_deg),
            yaw_rad=math.radians(args.head_yaw_deg),
            speed_percent=args.body_speed_percent,
            wait=True,
            timeout_sec=args.motion_timeout,
        )
        sleep_fn(STEP_INTERVAL_SEC)

        print("waypoint 1, step 4/8: move waist")
        robot.body.move_waist(
            pitch_rad=math.radians(args.waist_pitch_deg),
            lift_mm=args.waist_lift_mm,
            speed_percent=args.body_speed_percent,
            wait=True,
            timeout_sec=args.motion_timeout,
        )
        sleep_fn(STEP_INTERVAL_SEC)

        print("waypoint 1, step 5/8: move both arms with MoveP local offset")
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
        sleep_fn(STEP_INTERVAL_SEC)

        print("waypoint 1, step 6/8: move both arms with MoveL local offset")
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
        sleep_fn(STEP_INTERVAL_SEC)

        print("waypoint 1, step 7/8: open both grippers")
        robot.gripper.open(
            ArmSide.DUAL,
            wait=True,
            timeout_sec=args.motion_timeout,
        )
        sleep_fn(STEP_INTERVAL_SEC)

        print("waypoint 1, step 8/8: close both grippers")
        robot.gripper.close(
            ArmSide.DUAL,
            wait=True,
            timeout_sec=args.motion_timeout,
        )
        sleep_fn(STEP_INTERVAL_SEC)

        print("stage 6: navigate to waypoint 2")
        robot.navigation.clear_local_costmap(
            timeout_sec=args.server_timeout
        )
        robot.navigation.clear_global_costmap(
            timeout_sec=args.server_timeout
        )
        waypoint_2_navigation = robot.navigation.navigate_waypoints(
            (2,),
            path=args.file,
            server_timeout_sec=args.server_timeout,
        )
        waypoint_2_result = waypoint_2_navigation.wait(
            timeout_sec=args.navigation_timeout,
            cancel_on_timeout=True,
            cancel_timeout_sec=args.server_timeout,
        )
        print(f"waypoint 2 navigation finished: {waypoint_2_result.status}")
        sleep_fn(STEP_INTERVAL_SEC)

        print("waypoint 2, step 1/8: move left arm with MoveJ")
        robot.arms.move_j(
            left=tuple(
                math.radians(value) for value in args.left_joints_deg
            ),
            speed_rad_s=args.arm_velocity,
            acceleration_rad_s2=args.arm_acceleration,
            wait=True,
            timeout_sec=args.motion_timeout,
        )
        sleep_fn(STEP_INTERVAL_SEC)

        print("waypoint 2, step 2/8: move right arm with MoveJ")
        robot.arms.move_j(
            right=tuple(
                math.radians(value) for value in args.right_joints_deg
            ),
            speed_rad_s=args.arm_velocity,
            acceleration_rad_s2=args.arm_acceleration,
            wait=True,
            timeout_sec=args.motion_timeout,
        )
        sleep_fn(STEP_INTERVAL_SEC)

        print("waypoint 2, step 3/8: move head")
        robot.body.move_head(
            pitch_rad=math.radians(args.head_pitch_deg),
            yaw_rad=math.radians(args.head_yaw_deg),
            speed_percent=args.body_speed_percent,
            wait=True,
            timeout_sec=args.motion_timeout,
        )
        sleep_fn(STEP_INTERVAL_SEC)

        print("waypoint 2, step 4/8: move waist")
        robot.body.move_waist(
            pitch_rad=math.radians(args.waist_pitch_deg),
            lift_mm=args.waist_lift_mm,
            speed_percent=args.body_speed_percent,
            wait=True,
            timeout_sec=args.motion_timeout,
        )
        sleep_fn(STEP_INTERVAL_SEC)

        print("waypoint 2, step 5/8: move both arms with MoveP local offset")
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
        sleep_fn(STEP_INTERVAL_SEC)

        print("waypoint 2, step 6/8: move both arms with MoveL local offset")
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
        sleep_fn(STEP_INTERVAL_SEC)

        print("waypoint 2, step 7/8: open both grippers")
        robot.gripper.open(
            ArmSide.DUAL,
            wait=True,
            timeout_sec=args.motion_timeout,
        )
        sleep_fn(STEP_INTERVAL_SEC)

        print("waypoint 2, step 8/8: close both grippers")
        robot.gripper.close(
            ArmSide.DUAL,
            wait=True,
            timeout_sec=args.motion_timeout,
        )
        sleep_fn(STEP_INTERVAL_SEC)

        print(f"stage 7: announce completion: {args.speech_text}")
        robot.voice.speak(
            args.speech_text,
            wait=True,
            speech_timeout_sec=args.speech_timeout,
            call_timeout_sec=args.speech_call_timeout,
        )
    except BaseException:
        task_failed = True
        raise
    finally:
        cleanup_errors = []

        print("stage 8: cancel any active navigation")
        try:
            robot.navigation.cancel_all(timeout_sec=args.service_timeout)
        except BaseException as exc:
            cleanup_errors.append(exc)

        if enable_requested:
            print("stage 9: Hold both arms")
            try:
                robot.arms.hold(timeout_sec=args.service_timeout)
            except BaseException as exc:
                cleanup_errors.append(exc)

        if auto_requested:
            print("stage 10: switch System to IDLE")
            try:
                robot.system.set_control_mode(
                    ControlMode.IDLE,
                    timeout_sec=args.service_timeout,
                )
                robot.system.wait_for_control_mode(
                    ControlMode.IDLE,
                    timeout_sec=args.state_timeout,
                )
            except BaseException as exc:
                cleanup_errors.append(exc)

        if enable_requested:
            print("stage 11: disable both arms, head, and waist")
            try:
                robot.system.disable_all(timeout_sec=args.service_timeout)
                robot.system.wait_for_all_disabled(
                    timeout_sec=args.state_timeout
                )
            except BaseException as exc:
                cleanup_errors.append(exc)

        if cleanup_errors:
            for cleanup_error in cleanup_errors:
                print(f"cleanup failed: {cleanup_error}", file=sys.stderr)
            if not task_failed:
                raise cleanup_errors[0]


def main() -> int:
    """创建独立 ROS 2 运行时并执行完整移动操作流程。"""
    parser = build_parser()
    args = parser.parse_args()
    validate_arguments(parser, args)
    try:
        with Robot.standalone(
            node_name="sdk_mobile_manipulation_demo"
        ) as robot:
            run_demo(robot, args)
        print("mobile manipulation demo completed; all body axes disabled")
        return 0
    except KeyboardInterrupt:
        print(
            "mobile manipulation demo interrupted after safety cleanup",
            file=sys.stderr,
        )
        return 130
    except (OSError, ValueError, WheelloongSdkError) as exc:
        print(f"mobile manipulation demo failed: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
