#!/usr/bin/env python3
"""交互式读取机器人当前位置并保存为编号导航点位。

本例不发送运动命令。定位和 TF 正常时，程序持续读取 `map -> base_link`，按 S
立即把当前 x、y、yaw 追加到点位文件，按 Q 退出；可通过命令行参数修改点位文件
以及全局和机器人坐标系名称。
"""

import argparse
import math
from pathlib import Path
import sys
import termios
import threading
import tty

import rclpy
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from wheelloong_auto_sdk import (
    NavigationPose,
    WheelloongSdkError,
    append_waypoint,
    default_waypoint_path,
    load_waypoints,
)


SOURCE_WAYPOINT_PATH = (
    Path(__file__).resolve().parents[1]
    / "waypoint"
    / "waypoints.txt"
)


def build_parser() -> argparse.ArgumentParser:
    """Create command-line options for frames, TF timeout, and output file."""
    parser = argparse.ArgumentParser(
        description="Press S to append map->base_link poses to waypoints.txt."
    )
    parser.add_argument(
        "--file",
        type=Path,
        default=default_waypoint_path(fallback_path=SOURCE_WAYPOINT_PATH),
    )
    parser.add_argument("--global-frame", default="map")
    parser.add_argument("--base-frame", default="base_link")
    parser.add_argument("--tf-timeout", type=float, default=1.0)
    return parser


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """Return planar yaw in radians from an xyzw quaternion."""
    sin_yaw = 2.0 * (w * z + x * y)
    cos_yaw = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(sin_yaw, cos_yaw)


def pose_from_transform(transform, frame_id: str) -> NavigationPose:
    """Convert a map-to-base transform into a planar navigation pose."""
    translation = transform.transform.translation
    rotation = transform.transform.rotation
    return NavigationPose(
        x_m=translation.x,
        y_m=translation.y,
        yaw_rad=yaw_from_quaternion(
            rotation.x, rotation.y, rotation.z, rotation.w
        ),
        frame_id=frame_id,
    ).validated()


def read_single_key() -> str:
    """Read one terminal key immediately without requiring Enter."""
    return sys.stdin.read(1)


def run_recorder(args: argparse.Namespace) -> None:
    """Spin TF in the background and process S/Q terminal keys."""
    if args.tf_timeout <= 0.0:
        raise ValueError("--tf-timeout must be greater than zero")
    if not sys.stdin.isatty():
        message = "waypoint_recorder.py requires an interactive terminal"
        raise RuntimeError(message)

    rclpy.init()
    node = Node("wheelloong_waypoint_recorder")
    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer, node)
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    terminal_fd = sys.stdin.fileno()
    terminal_settings = termios.tcgetattr(terminal_fd)

    try:
        existing = load_waypoints(args.file)
        print(f"waypoint file: {args.file}")
        print(f"stored ids: {list(existing) if existing else 'none'}")
        print("press S to save map->base_link; press Q to quit")
        tty.setcbreak(terminal_fd)
        while rclpy.ok():
            key = read_single_key().lower()
            if key == "q":
                print("\nwaypoint recorder stopped")
                break
            if key != "s":
                continue
            try:
                transform = tf_buffer.lookup_transform(
                    args.global_frame,
                    args.base_frame,
                    Time(),
                    timeout=Duration(seconds=args.tf_timeout),
                )
                pose = pose_from_transform(transform, args.global_frame)
                waypoint = append_waypoint(pose, path=args.file)
                print(
                    "\nsaved id={} x={:.3f} y={:.3f} yaw={:.3f} rad".format(
                        waypoint.waypoint_id,
                        pose.x_m,
                        pose.y_m,
                        pose.yaw_rad,
                    )
                )
            except (TransformException, WheelloongSdkError, OSError) as exc:
                print(f"\nwaypoint not saved: {exc}", file=sys.stderr)
    finally:
        termios.tcsetattr(terminal_fd, termios.TCSADRAIN, terminal_settings)
        executor.shutdown()
        spin_thread.join(timeout=2.0)
        del tf_listener
        node.destroy_node()
        # Ctrl+C may have closed the Context through rclpy's signal handler.
        # try_shutdown() is safe for both an active and an inactive Context.
        rclpy.try_shutdown()


def main() -> int:
    """Run the recorder and translate failures to exit codes."""
    try:
        run_recorder(build_parser().parse_args())
    except KeyboardInterrupt:
        print("\nwaypoint recorder interrupted")
    except Exception as exc:
        print(f"waypoint recorder failed: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
