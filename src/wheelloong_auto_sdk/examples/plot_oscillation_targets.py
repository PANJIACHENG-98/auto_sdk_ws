#!/usr/bin/env python3
"""读取四个离线目标 TXT 并绘制四组目标曲线。"""

import argparse
import csv
from pathlib import Path
from typing import Dict, List, Tuple

import matplotlib


matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402


def build_parser() -> argparse.ArgumentParser:
    """创建输入目录和 PNG 输出路径解析器。"""
    default_dir = Path.cwd() / "simulated_targets"
    parser = argparse.ArgumentParser(
        description="Plot the four simulated target TXT files.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--input-dir", type=Path, default=default_dir)
    parser.add_argument(
        "--output",
        type=Path,
        default=default_dir / "oscillation_targets.png",
    )
    return parser


def read_table(path: Path) -> Tuple[List[str], Dict[str, List[float]]]:
    """读取带表头的逗号分隔 TXT 并转换为浮点列。

    Args:
        path: 一个模拟目标 TXT 文件。
    Returns:
        原始列名和按列名索引的浮点数据。
    Raises:
        ValueError: 文件缺少表头、数据行或包含非数字内容。
    """
    with path.open("r", encoding="utf-8", newline="") as file:
        reader = csv.DictReader(file)
        headers = reader.fieldnames
        if not headers:
            raise ValueError(f"missing header: {path}")
        columns = {name: [] for name in headers}
        for row in reader:
            for name in headers:
                columns[name].append(float(row[name]))
    if not columns["task_elapsed_sec"]:
        raise ValueError(f"no target rows: {path}")
    return headers, columns


def plot_arm(axis, path: Path, title: str) -> None:
    """在指定子图绘制一侧机械臂七关节目标。"""
    headers, columns = read_table(path)
    time_sec = columns["task_elapsed_sec"]
    for name in headers[1:]:
        axis.plot(time_sec, columns[name], label=name.removesuffix("_rad"))
    axis.set_title(title)
    axis.set_ylabel("target (rad)")
    axis.grid(True, alpha=0.3)
    axis.legend(ncol=2, fontsize=8)


def plot_head(axis, path: Path) -> None:
    """绘制头部俯仰和旋转目标。"""
    _, columns = read_table(path)
    time_sec = columns["task_elapsed_sec"]
    axis.plot(time_sec, columns["head_pitch_rad"], label="head pitch")
    axis.plot(time_sec, columns["head_yaw_rad"], "--", label="head yaw")
    axis.set_title("Head targets")
    axis.set_ylabel("target (rad)")
    axis.grid(True, alpha=0.3)
    axis.legend()


def plot_waist(axis, path: Path) -> None:
    """用双纵轴分别绘制腰部俯仰角和升降毫米值。"""
    _, columns = read_table(path)
    time_sec = columns["task_elapsed_sec"]
    pitch_line = axis.plot(
        time_sec,
        columns["waist_pitch_rad"],
        color="tab:blue",
        label="waist pitch",
    )
    lift_axis = axis.twinx()
    lift_line = lift_axis.plot(
        time_sec,
        columns["waist_lift_mm"],
        color="tab:red",
        label="waist lift",
    )
    axis.set_title("Waist targets")
    axis.set_ylabel("pitch target (rad)", color="tab:blue")
    lift_axis.set_ylabel("lift target (mm)", color="tab:red")
    axis.grid(True, alpha=0.3)
    axis.legend(pitch_line + lift_line, ["waist pitch", "waist lift"])


def create_plot(input_dir: Path, output_path: Path) -> Path:
    """读取四个目标文件并生成一个 2×2 总览图。"""
    figure, axes = plt.subplots(2, 2, figsize=(14, 9), sharex=True)
    plot_arm(
        axes[0, 0], input_dir / "left_arm_targets.txt", "Left arm targets"
    )
    plot_arm(
        axes[0, 1], input_dir / "right_arm_targets.txt", "Right arm targets"
    )
    plot_head(axes[1, 0], input_dir / "head_targets.txt")
    plot_waist(axes[1, 1], input_dir / "waist_targets.txt")
    for axis in axes[1, :]:
        axis.set_xlabel("elapsed from task start (s)")
    figure.suptitle("Wheelloong AUTO oscillation target simulation")
    figure.tight_layout()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output_path, dpi=160)
    plt.close(figure)
    return output_path


def main() -> int:
    """解析路径、绘制目标曲线并打印 PNG 位置。"""
    args = build_parser().parse_args()
    output_path = create_plot(args.input_dir.resolve(), args.output.resolve())
    print(f"plot: {output_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
