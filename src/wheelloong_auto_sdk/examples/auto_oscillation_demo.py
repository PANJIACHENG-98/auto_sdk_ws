#!/usr/bin/env python3
"""使用 SDK 复现官方 auto_client.py 的 AUTO 振荡测试。

该程序会真实控制双臂、头部、腰部和可选双夹爪，但不会控制底盘。
流程包括 AUTO/使能、工作姿态、正弦振荡、结束姿态、Idle/去使能。
所有机器人调用都经过 wheelloong_auto_sdk 公共 API，不直接构造 ROS 消息。
运行前必须确认机器人已正常启动、周围无人且不存在运动干涉风险。
"""

import argparse
import math
import sys
import time
from typing import Sequence, Tuple

from wheelloong_auto_sdk import (
    ArmSide,
    AxisSelection,
    CartesianPoseMM,
    Robot,
    WheelloongSdkError,
    get_robot_profile,
)
from wheelloong_auto_sdk._cli import (
    boolean_switch,
    non_negative_number,
    positive_number,
)


# 与当前 install 版官方 Demo 一致：腰部从零点向负方向下降 100 mm。
# 该常量是正的“幅度”，实际目标由下方公式取负后发送为 -100 mm。
LUMBAR_LIFT_AMPLITUDE_MM = 100.0

def build_parser() -> argparse.ArgumentParser:
    """创建与官方 oscillation-test 参数语义一致的解析器。

    Returns:
        包含机器人类型、轨迹、安全限制和超时参数的解析器。
    Notes:
        参数默认值保持与官方 Demo 一致，便于逐项对比请求行为。
    """
    parser = argparse.ArgumentParser(
        description="Run the official AUTO oscillation flow through the SDK.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--robot",
        choices=("wheelloong", "shiloong"),
        required=True,
        help="robot model; selects the final dual-arm pose",
    )
    # duration=0 表示一直运行到 Ctrl+C；period 是完成一个正弦周期的时间。
    parser.add_argument(
        "--duration",
        type=non_negative_number,
        default=20.0,
        help="oscillation duration in seconds; 0 runs until Ctrl+C",
    )
    parser.add_argument(
        "--period",
        type=positive_number,
        default=4.0,
        help="seconds per complete sine-wave cycle",
    )
    # rate 是服务目标下发频率，不是关节自身控制器的闭环频率。
    parser.add_argument(
        "--rate",
        type=positive_number,
        default=5.0,
        help="sampled service-command rate in Hz; maximum 20",
    )
    parser.add_argument(
        "--amplitude-deg",
        type=positive_number,
        default=3.0,
        help="shared arm/head/waist angular amplitude in degrees",
    )
    parser.add_argument(
        "--lumbar-lift-amplitude-mm",
        type=non_negative_number,
        default=LUMBAR_LIFT_AMPLITUDE_MM,
        help="waist downward travel from zero in mm; maximum 100",
    )
    parser.add_argument(
        "--control-gripper",
        type=boolean_switch,
        default=True,
        help="whether both grippers oscillate between 0 and 1",
    )
    parser.add_argument(
        "--work-hold",
        "--zero-hold",
        dest="work_hold",
        type=non_negative_number,
        default=2.0,
        help="seconds to hold the work pose before oscillation",
    )
    # state-timeout 用于模式/使能确认；service-timeout 用于普通服务响应。
    parser.add_argument(
        "--state-timeout",
        type=positive_number,
        default=10.0,
        help="seconds to confirm AUTO, IDLE, enable, and disable state",
    )
    parser.add_argument(
        "--service-timeout",
        type=positive_number,
        default=3.0,
        help="seconds to wait for each sampled service response",
    )
    parser.add_argument(
        "--motion-timeout",
        type=positive_number,
        default=60.0,
        help="seconds allowed for blocking work/final arm motion",
    )
    # 0 表示使用底层默认值；当前 Nero MoveJ 默认速度为 0.628 rad/s（20%）。
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
    return parser


def validate_arguments(
    parser: argparse.ArgumentParser, args: argparse.Namespace
) -> None:
    """应用与官方 Demo 相同的硬件测试安全上限。

    Args:
        parser: 用于报告参数错误并终止运行的解析器。
        args: 已完成类型转换的命令行参数。
    Raises:
        SystemExit: 频率、角度或腰部升降振幅超过测试上限。
    """
    if args.rate > 20.0:
        parser.error("--rate must not exceed 20 Hz")
    if args.amplitude_deg > 10.0:
        parser.error("--amplitude-deg must not exceed 10 degrees")
    if args.lumbar_lift_amplitude_mm > LUMBAR_LIFT_AMPLITUDE_MM:
        parser.error("--lumbar-lift-amplitude-mm must not exceed 100")


def oscillating_arm_targets(
    robot_name: str, offset_rad: float
) -> Tuple[Sequence[float], Sequence[float]]:
    """把同一正弦偏移叠加到左右臂各自的七轴工作姿态。"""
    left_center, right_center = get_robot_profile(robot_name).work_arm_joints()
    return (
        tuple(center + offset_rad for center in left_center),
        tuple(center + offset_rad for center in right_center),
    )


def command_test_pose(
    robot: Robot,
    args: argparse.Namespace,
    left: Sequence[float],
    right: Sequence[float],
    body_targets: Sequence[float],
    gripper_position: float,
    *,
    blocking: bool,
) -> None:
    """按官方顺序下发一组全身测试目标。

    Args:
        robot/args: SDK 对象和本次运行参数。
        left/right: 左右臂七关节绝对目标，单位 rad。
        body_targets: 头俯仰、头旋转、腰俯仰(rad)和腰升降(mm)。
        gripper_position: 双夹爪归一化位置，范围 [0, 1]。
        blocking: 是否等待手臂和头腰实际到达目标。
    Raises:
        ValidationError: body_targets 不是严格的四元素序列。
    """
    if len(body_targets) != 4:
        raise ValidationError("body_targets must contain exactly four values")
    (
        head_pitch_rad,
        head_yaw_rad,
        waist_pitch_rad,
        waist_lift_mm,
    ) = body_targets

    # 工作/结束姿态需要等待完成；连续振荡只等待服务接收请求。
    arm_timeout = args.motion_timeout if blocking else args.service_timeout
    # JntsCtrl 底层只接受不超过 30 秒的阻塞运动超时。
    body_timeout = min(30.0, arm_timeout)

    # MoveJ 使用绝对模式。velocity/acceleration=0 沿用 Nero 驱动默认值。
    robot.arms.move_j(
        left=left,
        right=right,
        speed_rad_s=args.arm_velocity,
        acceleration_rad_s2=args.arm_acceleration,
        wait=blocking,
        timeout_sec=arm_timeout,
    )
    # 四个自由度分别传入；即使目标相同也不在这里隐式复用某个值。
    # speed_percent=0 与官方请求一致；当前头腰底层暂未使用速度字段。
    robot.body.move(
        head_pitch_rad=head_pitch_rad,
        head_yaw_rad=head_yaw_rad,
        waist_pitch_rad=waist_pitch_rad,
        waist_lift_mm=waist_lift_mm,
        speed_percent=0.0,
        wait=blocking,
        timeout_sec=body_timeout,
    )
    if args.control_gripper:
        # 夹爪跟随每个采样点，不等待实际位置，否则无法保持 5 Hz 下发。
        # speed/torque=0 表示使用夹爪驱动默认值，与官方 Demo 一致。
        robot.gripper.set(
            ArmSide.DUAL,
            gripper_position,
            speed=0.0,
            torque=0.0,
            wait=False,
            timeout_sec=args.service_timeout,
        )


def run_oscillation_samples(
    robot: Robot, args: argparse.Namespace, axes: AxisSelection
) -> None:
    """按固定频率产生并下发官方 Demo 的正弦轨迹。

    Args:
        robot/args: 已进入 AUTO 的机器人和轨迹参数。
        axes: 每个采样点都要重新确认的轴使能集合。
    Notes:
        双臂围绕各自工作姿态振荡；头腰角度围绕零点振荡。
        升降为 0 到负振幅，夹爪为 0 到 1。
        使用单调时钟和绝对采样时刻，避免系统时间变化及累计漂移。
    """
    # monotonic 不受系统校时影响，适合计算持续时间和控制周期。
    started_at = time.monotonic()
    next_sample = started_at
    interval = 1.0 / args.rate
    while (
        args.duration == 0.0
        or time.monotonic() - started_at < args.duration
    ):
        # 前端切换 Manual 或轴失能时立即停止继续下发采样目标。
        robot.system.require_ready(
            axes,
            max_age_sec=1.0,
            wait_timeout_sec=1.0,
        )
        elapsed = time.monotonic() - started_at

        # phase 每经过 period 秒增加 2π，形成一个完整振荡周期。
        phase = 2.0 * math.pi * elapsed / args.period
        # angle 范围为 [-amplitude, +amplitude]，下发前由度转换为 rad。
        angle = math.radians(args.amplitude_deg) * math.sin(phase)
        # cycle=(1-cos)/2 范围 [0,1]，用于夹爪开合和腰部单向下降。
        # 默认 T=4s：0s 为 0/0mm，1s 为 +3°/-50mm，2s 为 0°/-100mm，
        # 3s 为 -3°/-50mm，4s 回到 0°/0mm；夹爪同期为 0/.5/1/.5/0。
        cycle = 0.5 * (1.0 - math.cos(phase))
        left_target, right_target = oscillating_arm_targets(args.robot, angle)
        command_test_pose(
            robot=robot,
            args=args,
            left=left_target,
            right=right_target,
            body_targets=[
                angle,  # 头部俯仰，rad
                angle,  # 头部旋转，rad
                angle,  # 腰部俯仰，rad
                -args.lumbar_lift_amplitude_mm * cycle,  # 腰部升降，mm
            ],
            gripper_position=cycle,
            blocking=False,
        )
        # 用上一目标时刻累加周期，而不是从当前时间重新计时，减少漂移。
        next_sample += interval
        remaining = next_sample - time.monotonic()
        if remaining > 0.0:
            time.sleep(remaining)


def command_final_pose(robot: Robot, args: argparse.Namespace) -> None:
    """停止采样运动并到达机器人对应的安全结束姿态。

    Args:
        robot: 当前 AUTO 会话中的机器人对象。
        args: 提供机器人型号、速度和超时参数。
    Notes:
        先发送 Hold，再以阻塞方式运动；头腰和夹爪同时回到零。
    """
    # Hold 先终止最后一条非阻塞采样轨迹，防止它影响结束姿态。
    robot.arms.hold(timeout_sec=args.service_timeout)
    left, right = get_robot_profile(args.robot).return_arm_joints()
    command_test_pose(
        robot=robot,
        args=args,
        left=left,
        right=right,
        body_targets=[
            0.0,  # 头部俯仰，rad
            0.0,  # 头部旋转，rad
            0.0,  # 腰部俯仰，rad
            0.0,  # 腰部升降，mm
        ],
        gripper_position=0.0,
        blocking=True,
    )


def run_demo(robot: Robot, args: argparse.Namespace) -> None:
    """在 AUTO 会话中执行振荡、MoveP、MoveL 和结束姿态任务。"""
    axes = AxisSelection.all()
    profile = get_robot_profile(args.robot)

    print("stage 1/7: enter AUTO session and enable all motion axes")
    with robot.auto_session(
        required_axes=axes,
        state_timeout_sec=args.state_timeout,
        service_timeout_sec=args.service_timeout,
    ):
        print("stage 2/7: move arms to work pose and head/waist to zero")
        work_left, work_right = profile.work_arm_joints()
        command_test_pose(
            robot=robot,
            args=args,
            left=work_left,
            right=work_right,
            body_targets=(0.0, 0.0, 0.0, 0.0),
            gripper_position=0.0,
            blocking=True,
        )
        if args.work_hold > 0.0:
            time.sleep(args.work_hold)

        print("stage 3/7: oscillate arms, head, and waist; chassis untouched")
        oscillation_interrupted = False
        try:
            run_oscillation_samples(robot, args, axes)
        except KeyboardInterrupt:
            oscillation_interrupted = True
            print(
                "oscillation interrupted; skip MoveP/MoveL and use final pose"
            )

        if not oscillation_interrupted:
            print("stage 4/7: run MoveP local offset")
            robot.arms.move_p_offset(
                left=CartesianPoseMM.from_rpy_degrees(
                    0.0, 20.0, 30.0, 0.0, 0.0, 0.0
                ),
                right=CartesianPoseMM.from_rpy_degrees(
                    0.0, -20.0, 30.0, 0.0, 0.0, 0.0
                ),
                speed_rad_s=0.2,
                wait=True,
                timeout_sec=args.motion_timeout,
                hold_timeout_sec=args.service_timeout,
                state_timeout_sec=args.state_timeout,
            )

            print("stage 5/7: run MoveL local offset")
            robot.arms.move_l_offset(
                left=CartesianPoseMM.from_rpy_degrees(
                    0.0, -20.0, -30.0, 0.0, 0.0, 0.0
                ),
                right=CartesianPoseMM.from_rpy_degrees(
                    0.0, 20.0, -30.0, 0.0, 0.0, 0.0
                ),
                speed_mm_s=10.0,
                wait=True,
                timeout_sec=args.motion_timeout,
                hold_timeout_sec=args.service_timeout,
                state_timeout_sec=args.state_timeout,
            )
        else:
            print("stage 4/7: MoveP offset skipped after Ctrl+C")
            print("stage 5/7: MoveL offset skipped after Ctrl+C")

        print("stage 6/7: move to robot-specific final pose")
        command_final_pose(robot, args)
        print("stage 7/7: Hold, switch to IDLE, and disable all axes")


def main() -> int:
    """创建独立 SDK 运行时并执行测试。

    Returns:
        0 表示完整流程和清理成功，1 表示中断或 SDK 异常。
    Notes:
        Robot.standalone 自己管理 ROS Context、Node、Executor 及其线程。
    """
    parser = build_parser()
    args = parser.parse_args()
    validate_arguments(parser, args)
    try:
        # 外层 with 保证即使 AUTO 会话创建失败，也会释放 SDK 的 ROS 资源。
        with Robot.standalone(node_name="sdk_auto_oscillation_demo") as robot:
            run_demo(robot, args)
        print("AUTO oscillation demo completed successfully")
        return 0
    except KeyboardInterrupt:
        print(
            "AUTO oscillation demo interrupted after safety cleanup",
            file=sys.stderr,
        )
    except WheelloongSdkError as exc:
        print(f"AUTO oscillation demo failed: {exc}", file=sys.stderr)
    return 1


if __name__ == "__main__":
    sys.exit(main())
