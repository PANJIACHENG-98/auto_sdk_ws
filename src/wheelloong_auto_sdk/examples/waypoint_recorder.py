#!/usr/bin/env python3
"""交互读取当前导航位姿并保存为编号点位。

本例不发送运动命令。定位正常时按 S 保存当前 map -> base_link 位姿，按 Q 退出。
ROS Context、Node、Executor 和 TF 查询均由 SDK 管理。
"""

import argparse
from pathlib import Path
import sys

from wheelloong_auto_sdk import (
    Robot,
    WheelloongSdkError,
    append_waypoint,
    default_waypoint_path,
)
from wheelloong_auto_sdk._terminal import cbreak_terminal, read_key


def build_parser() -> argparse.ArgumentParser:
    """创建坐标系、TF 超时和点位输出文件参数。"""
    parser = argparse.ArgumentParser(
        description="Press S to save the current navigation pose; Q quits."
    )
    parser.add_argument("--file", type=Path, default=default_waypoint_path())
    parser.add_argument("--global-frame", default="map")
    parser.add_argument("--base-frame", default="base_link")
    parser.add_argument("--tf-timeout", type=float, default=1.0)
    return parser


def run_recorder(robot: Robot, args: argparse.Namespace) -> None:
    """响应 S/Q 按键，通过 SDK 读取并保存当前导航位姿。"""
    if args.tf_timeout <= 0.0:
        raise ValueError("--tf-timeout must be greater than zero")

    print(f"waypoint file: {args.file}")
    print("press S to save current pose; press Q to quit")
    with cbreak_terminal():
        while True:
            key = read_key().lower()
            if key == "q":
                print("\nwaypoint recorder stopped")
                return
            if key != "s":
                continue
            try:
                pose = robot.navigation.current_pose(
                    global_frame=args.global_frame,
                    base_frame=args.base_frame,
                    timeout_sec=args.tf_timeout,
                )
                waypoint = append_waypoint(pose, path=args.file)
                print(
                    "\nsaved id={} x={:.3f} y={:.3f} yaw={:.3f} rad".format(
                        waypoint.waypoint_id,
                        pose.x_m,
                        pose.y_m,
                        pose.yaw_rad,
                    )
                )
            except (WheelloongSdkError, OSError) as exc:
                print(f"\nwaypoint not saved: {exc}", file=sys.stderr)


def main() -> int:
    """创建独立 SDK 运行时并启动交互式点位记录任务。"""
    args = build_parser().parse_args()
    try:
        with Robot.standalone(node_name="sdk_waypoint_recorder") as robot:
            run_recorder(robot, args)
        return 0
    except KeyboardInterrupt:
        print("\nwaypoint recorder interrupted")
        return 130
    except Exception as exc:
        print(f"waypoint recorder failed: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
