#!/usr/bin/env python3
"""离线模拟 AUTO 振荡阶段并输出四个逗号分隔的 TXT。"""

import argparse
import csv
import math
from contextlib import ExitStack
from pathlib import Path
from typing import Dict, Iterator, Tuple


DEFAULT_DURATION_SEC = 20.0
DEFAULT_PERIOD_SEC = 4.0
DEFAULT_RATE_HZ = 5.0
DEFAULT_AMPLITUDE_DEG = 3.0
DEFAULT_LUMBAR_LIFT_MM = 100.0
SHILOONG_WORK_LEFT_DEG = (10.0, 77.0, -80.0, 20.0, -25.0, 10.0, 10.0)
SHILOONG_WORK_RIGHT_DEG = (-10.0, 77.0, 80.0, 20.0, 25.0, -10.0, 10.0)


def parse_positive(value: str) -> float:
    """解析有限正数命令行参数。"""
    parsed = float(value)
    if not math.isfinite(parsed) or parsed <= 0.0:
        raise argparse.ArgumentTypeError(f"expected finite value > 0: {value}")
    return parsed


def parse_non_negative(value: str) -> float:
    """解析有限非负命令行参数。"""
    parsed = float(value)
    if not math.isfinite(parsed) or parsed < 0.0:
        message = f"expected finite value >= 0: {value}"
        raise argparse.ArgumentTypeError(message)
    return parsed


def build_parser() -> argparse.ArgumentParser:
    """创建与振荡 Demo 默认轨迹参数对应的解析器。"""
    default_dir = Path.cwd() / "simulated_targets"
    parser = argparse.ArgumentParser(
        description=(
            "Simulate oscillation targets without ROS or robot commands."
        ),
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--duration", type=parse_positive, default=DEFAULT_DURATION_SEC
    )
    parser.add_argument(
        "--robot",
        choices=("wheelloong", "shiloong"),
        default="shiloong",
        help="robot model used to select the arm work pose",
    )
    parser.add_argument(
        "--period", type=parse_positive, default=DEFAULT_PERIOD_SEC
    )
    parser.add_argument("--rate", type=parse_positive, default=DEFAULT_RATE_HZ)
    parser.add_argument(
        "--amplitude-deg",
        type=parse_non_negative,
        default=DEFAULT_AMPLITUDE_DEG,
    )
    parser.add_argument(
        "--lumbar-lift-amplitude-mm",
        type=parse_non_negative,
        default=DEFAULT_LUMBAR_LIFT_MM,
    )
    parser.add_argument("--output-dir", type=Path, default=default_dir)
    return parser


def work_arm_targets(
    robot_name: str,
) -> Tuple[Tuple[float, ...], Tuple[float, ...]]:
    """返回离线模拟使用的左右臂七轴工作姿态，单位 rad。"""
    if robot_name == "shiloong":
        return (
            tuple(math.radians(value) for value in SHILOONG_WORK_LEFT_DEG),
            tuple(math.radians(value) for value in SHILOONG_WORK_RIGHT_DEG),
        )
    return (0.0,) * 7, (0.0,) * 7


def generate_samples(
    duration_sec: float,
    period_sec: float,
    rate_hz: float,
    amplitude_deg: float,
    lumbar_lift_amplitude_mm: float,
) -> Iterator[Tuple[float, float, float]]:
    """生成与实时循环一致的时间、角度和腰部升降目标。

    Yields:
        task_elapsed_sec、angle_rad、waist_lift_mm 三元组。
    Notes:
        程序任务启动为 0 秒，采样时刻严格小于 duration。
    """
    sample_index = 0
    amplitude_rad = math.radians(amplitude_deg)
    while True:
        task_elapsed_sec = sample_index / rate_hz
        if task_elapsed_sec >= duration_sec:
            return
        phase = 2.0 * math.pi * task_elapsed_sec / period_sec
        angle_rad = amplitude_rad * math.sin(phase)
        cycle = 0.5 * (1.0 - math.cos(phase))
        waist_lift_mm = -lumbar_lift_amplitude_mm * cycle
        yield task_elapsed_sec, angle_rad, waist_lift_mm
        sample_index += 1


def write_target_files(args: argparse.Namespace) -> Dict[str, Path]:
    """将四组目标写入独立 TXT 文件。

    Args:
        args: 已解析的轨迹参数和输出目录。
    Returns:
        从通道组名称到输出文件路径的映射。
    """
    output_dir = args.output_dir.resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    paths = {
        "left_arm": output_dir / "left_arm_targets.txt",
        "right_arm": output_dir / "right_arm_targets.txt",
        "head": output_dir / "head_targets.txt",
        "waist": output_dir / "waist_targets.txt",
    }

    with ExitStack() as stack:
        files = {
            name: stack.enter_context(
                path.open("w", encoding="utf-8", newline="")
            )
            for name, path in paths.items()
        }
        writers = {name: csv.writer(file) for name, file in files.items()}
        arm_header = ["task_elapsed_sec"] + [
            f"joint_{index}_rad" for index in range(1, 8)
        ]
        writers["left_arm"].writerow(arm_header)
        writers["right_arm"].writerow(arm_header)
        writers["head"].writerow(
            ["task_elapsed_sec", "head_pitch_rad", "head_yaw_rad"]
        )
        writers["waist"].writerow(
            ["task_elapsed_sec", "waist_pitch_rad", "waist_lift_mm"]
        )

        samples = generate_samples(
            args.duration,
            args.period,
            args.rate,
            args.amplitude_deg,
            args.lumbar_lift_amplitude_mm,
        )
        left_center, right_center = work_arm_targets(args.robot)
        for task_elapsed_sec, angle_rad, waist_lift_mm in samples:
            time_text = f"{task_elapsed_sec:.6f}"
            angle_text = f"{angle_rad:.9f}"
            left_row = [time_text] + [
                f"{center + angle_rad:.9f}" for center in left_center
            ]
            right_row = [time_text] + [
                f"{center + angle_rad:.9f}" for center in right_center
            ]
            writers["left_arm"].writerow(left_row)
            writers["right_arm"].writerow(right_row)
            writers["head"].writerow([time_text, angle_text, angle_text])
            writers["waist"].writerow(
                [time_text, angle_text, f"{waist_lift_mm:.6f}"]
            )
    return paths


def main() -> int:
    """生成默认或命令行指定的四组离线目标文件。"""
    args = build_parser().parse_args()
    paths = write_target_files(args)
    sample_count = sum(
        1
        for _ in generate_samples(
            args.duration,
            args.period,
            args.rate,
            args.amplitude_deg,
            args.lumbar_lift_amplitude_mm,
        )
    )
    print(f"generated {sample_count} {args.robot} samples per file")
    for name, path in paths.items():
        print(f"{name}: {path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
