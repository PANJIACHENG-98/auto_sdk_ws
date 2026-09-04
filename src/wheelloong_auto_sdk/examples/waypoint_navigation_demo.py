#!/usr/bin/env python3
"""Load stored waypoint ids and call the SDK navigation API directly."""

import argparse
from pathlib import Path
import sys

from wheelloong_auto_sdk import (
    ControlMode,
    Robot,
    WheelloongSdkError,
    default_waypoint_path,
    load_waypoints,
)


SOURCE_WAYPOINT_PATH = (
    Path(__file__).resolve().parents[1]
    / "waypoint"
    / "waypoints.txt"
)


def positive_waypoint_id(value: str) -> int:
    """Parse one positive integer waypoint id for argparse."""
    try:
        waypoint_id = int(value)
    except ValueError as exc:
        message = "waypoint id must be an integer"
        raise argparse.ArgumentTypeError(message) from exc
    if waypoint_id < 1 or str(waypoint_id) != value.strip():
        message = "waypoint id must be greater than zero"
        raise argparse.ArgumentTypeError(message)
    return waypoint_id


def build_parser() -> argparse.ArgumentParser:
    """Create waypoint id, file, and navigation timeout options."""
    parser = argparse.ArgumentParser(
        description=(
            "Navigate stored ids; one uses navigate_to, many use through."
        )
    )
    parser.add_argument("waypoint_ids", nargs="+", type=positive_waypoint_id)
    parser.add_argument(
        "--file",
        type=Path,
        default=default_waypoint_path(fallback_path=SOURCE_WAYPOINT_PATH),
    )
    parser.add_argument("--navigation-timeout", type=float, default=300.0)
    parser.add_argument("--server-timeout", type=float, default=5.0)
    return parser


def main() -> int:
    """Load stored poses and call the public navigation API directly."""
    args = build_parser().parse_args()
    if min(args.server_timeout, args.navigation_timeout) <= 0.0:
        print("navigation timeouts must be greater than zero", file=sys.stderr)
        return 2
    try:
        waypoints = load_waypoints(args.file)
        missing = [
            identifier
            for identifier in args.waypoint_ids
            if identifier not in waypoints
        ]
        if missing:
            values = ",".join(str(identifier) for identifier in missing)
            raise ValueError(f"unknown waypoint ids: {values}")
        poses = [
            waypoints[identifier].pose
            for identifier in args.waypoint_ids
        ]
        for identifier, pose in zip(args.waypoint_ids, poses):
            print(
                f"target {identifier}: x={pose.x_m:.3f} m, "
                f"y={pose.y_m:.3f} m, yaw={pose.yaw_rad:.3f} rad, "
                f"frame={pose.frame_id}"
            )

        with Robot.standalone(
            node_name="sdk_waypoint_navigation_demo"
        ) as robot:
            handle = None
            auto_requested = False
            primary_error = None
            initial_state = robot.system.state(
                wait_timeout_sec=args.server_timeout
            )
            restore_idle = (
                initial_state.control_mode != int(ControlMode.AUTO)
            )
            print(f"initial control mode: {initial_state.control_mode}")
            try:
                try:
                    # 第一步：进入 AUTO 并确认；导航不调用任何轴使能接口。
                    print(
                        "switching control mode to AUTO "
                        "(no actuator enable)"
                    )
                    robot.system.set_control_mode(
                        ControlMode.AUTO,
                        timeout_sec=args.server_timeout,
                    )
                    auto_requested = True
                    robot.system.wait_for_control_mode(
                        ControlMode.AUTO,
                        timeout_sec=args.server_timeout,
                    )
                    print("AUTO confirmed; no enable command was sent")

                    current_pose = robot.navigation.current_pose(
                        timeout_sec=args.server_timeout
                    )
                    print(
                        "current pose: "
                        f"x={current_pose.x_m:.3f} m, "
                        f"y={current_pose.y_m:.3f} m, "
                        f"yaw={current_pose.yaw_rad:.3f} rad, "
                        f"frame={current_pose.frame_id}"
                    )

                    # 第二步：分别清除局部和全局代价地图中的历史障碍数据。
                    print("clearing local costmap")
                    robot.navigation.clear_local_costmap(
                        timeout_sec=args.server_timeout
                    )
                    print("clearing global costmap")
                    robot.navigation.clear_global_costmap(
                        timeout_sec=args.server_timeout
                    )

                    # 第三步：下发非阻塞目标，再在 Demo 末尾等待终态。
                    if len(poses) == 1:
                        print(f"calling navigate_to: {args.waypoint_ids[0]}")
                        handle = robot.navigation.navigate_to(
                            poses[0], server_timeout_sec=args.server_timeout
                        )
                    else:
                        values = ",".join(
                            str(identifier) for identifier in args.waypoint_ids
                        )
                        print(f"calling navigate_through: {values}")
                        handle = robot.navigation.navigate_through(
                            poses, server_timeout_sec=args.server_timeout
                        )

                    # 等待目标结束；超时会先取消并确认最终状态。
                    result = handle.wait(
                        timeout_sec=args.navigation_timeout,
                        cancel_on_timeout=True,
                        cancel_timeout_sec=args.server_timeout,
                    )
                except KeyboardInterrupt:
                    print("\nCtrl+C received; stopping navigation")
                    if handle is None:
                        print("no navigation goal was accepted")
                    elif handle.done:
                        print("navigation goal already reached a final state")
                    else:
                        try:
                            cancel_result = handle.cancel(
                                timeout_sec=args.server_timeout
                            )
                            print(
                                "navigation cancellation confirmed: "
                                f"{cancel_result.status}"
                            )
                        except WheelloongSdkError as cancel_error:
                            print(
                                "navigation cancellation failed: "
                                f"{cancel_error}",
                                file=sys.stderr,
                            )
                    raise
            except BaseException as error:
                primary_error = error
                raise
            finally:
                # 只恢复由本 Demo 改变的模式；从 AUTO 启动则保持 AUTO。
                if auto_requested and restore_idle:
                    try:
                        print(
                            "restoring control mode to IDLE "
                            "(no disable command)"
                        )
                        robot.system.set_control_mode(
                            ControlMode.IDLE,
                            timeout_sec=args.server_timeout,
                        )
                        robot.system.wait_for_control_mode(
                            ControlMode.IDLE,
                            timeout_sec=args.server_timeout,
                        )
                        print("IDLE confirmed; no disable command was sent")
                    except WheelloongSdkError as cleanup_error:
                        if primary_error is None:
                            raise
                        print(
                            "failed to restore IDLE after navigation error: "
                            f"{cleanup_error}",
                            file=sys.stderr,
                        )
        print(f"navigation finished: {result.status}")
    except KeyboardInterrupt:
        print("navigation demo interrupted", file=sys.stderr)
        return 130
    except (OSError, ValueError, WheelloongSdkError) as exc:
        print(f"navigation failed: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
