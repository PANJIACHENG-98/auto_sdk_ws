#!/usr/bin/env python3
"""把点位编号序列交给 SDK，并等待单点或多点导航完成。

本例会真实驱动机器人底盘，但不会给双臂、头部或腰部上使能。SDK 负责读取点位、
选择单点/多点 Action、进入 AUTO、取消活动目标和恢复进入前的控制模式。
"""

import argparse
from pathlib import Path
import sys

from wheelloong_auto_sdk import (
    AxisSelection,
    Robot,
    WheelloongSdkError,
    default_waypoint_path,
)


def positive_waypoint_id(value: str) -> int:
    """解析一个正整数导航点编号。"""
    try:
        waypoint_id = int(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(
            "waypoint id must be an integer"
        ) from exc
    if waypoint_id < 1 or str(waypoint_id) != value.strip():
        raise argparse.ArgumentTypeError(
            "waypoint id must be greater than zero"
        )
    return waypoint_id


def build_parser() -> argparse.ArgumentParser:
    """创建点位编号、点位文件和导航超时参数。"""
    parser = argparse.ArgumentParser(
        description="Navigate a sequence of stored waypoint ids."
    )
    parser.add_argument("waypoint_ids", nargs="+", type=positive_waypoint_id)
    parser.add_argument("--file", type=Path, default=default_waypoint_path())
    parser.add_argument("--navigation-timeout", type=float, default=300.0)
    parser.add_argument("--server-timeout", type=float, default=5.0)
    return parser


def main() -> int:
    """将编号序列交给公共导航接口并等待任务结束。"""
    args = build_parser().parse_args()
    if min(args.server_timeout, args.navigation_timeout) <= 0.0:
        print("navigation timeouts must be greater than zero", file=sys.stderr)
        return 2

    try:
        print(f"waypoint sequence: {args.waypoint_ids}")
        with Robot.standalone(
            node_name="sdk_waypoint_navigation_demo"
        ) as robot:
            with robot.auto_session(
                required_axes=AxisSelection.none(),
                state_timeout_sec=args.server_timeout,
                service_timeout_sec=args.server_timeout,
                restore_initial_mode=True,
            ):
                current = robot.navigation.current_pose(
                    timeout_sec=args.server_timeout
                )
                print(
                    f"current pose: x={current.x_m:.3f} m, "
                    f"y={current.y_m:.3f} m, yaw={current.yaw_rad:.3f} rad"
                )
                robot.navigation.clear_local_costmap(
                    timeout_sec=args.server_timeout
                )
                robot.navigation.clear_global_costmap(
                    timeout_sec=args.server_timeout
                )
                handle = robot.navigation.navigate_waypoints(
                    args.waypoint_ids,
                    path=args.file,
                    server_timeout_sec=args.server_timeout,
                )
                result = handle.wait(
                    timeout_sec=args.navigation_timeout,
                    cancel_on_timeout=True,
                    cancel_timeout_sec=args.server_timeout,
                )
        print(f"navigation finished: {result.status}")
        return 0
    except KeyboardInterrupt:
        print("navigation interrupted; active goal cancellation requested")
        return 130
    except (OSError, ValueError, WheelloongSdkError) as exc:
        print(f"navigation failed: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
