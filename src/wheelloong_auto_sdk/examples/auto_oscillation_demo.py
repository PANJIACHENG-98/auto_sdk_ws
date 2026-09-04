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
    ControlMode,
    Robot,
    RobotStateError,
    ValidationError,
    WheelloongSdkError,
)


# 与当前 install 版官方 Demo 一致：腰部从零点向负方向下降 100 mm。
# 该常量是正的“幅度”，实际目标由下方公式取负后发送为 -100 mm。
LUMBAR_LIFT_AMPLITUDE_MM = 100.0

# 侍龙结束姿态和振荡中心均以度保存，真正下发前统一转换为 rad。
SHILOONG_RETURN_LEFT_DEG = (0.0, 85.0, -2.0, 3.0, -25.0, 0.0, 0.0)
SHILOONG_RETURN_RIGHT_DEG = (0.0, 85.0, -2.0, 3.0, 25.0, 0.0, 0.0)
SHILOONG_WORK_LEFT_DEG = (10.0, 77.0, -80.0, 20.0, -25.0, 10.0, 10.0)
SHILOONG_WORK_RIGHT_DEG = (-10.0, 77.0, 80.0, 20.0, 25.0, -10.0, 10.0)


def parse_switch(value: str) -> bool:
    """解析命令行布尔开关。

    Args:
        value: on/off、true/false、yes/no 或 1/0 字符串。
    Returns:
        对应的布尔值。
    Raises:
        ArgumentTypeError: 输入不属于任何支持的开关写法。
    """
    normalized = value.strip().lower()
    if normalized in ("on", "true", "1", "yes"):
        return True
    if normalized in ("off", "false", "0", "no"):
        return False
    raise argparse.ArgumentTypeError(f"expected on/off, got: {value}")


def parse_non_negative(value: str) -> float:
    """解析用于时间、振幅或速度的有限非负浮点数。

    Args:
        value: 命令行输入字符串，允许零。
    Returns:
        转换后的浮点数。
    Raises:
        ArgumentTypeError: 数值为负数、NaN 或无穷大。
    """
    parsed = float(value)
    if not math.isfinite(parsed) or parsed < 0.0:
        message = f"expected finite value >= 0: {value}"
        raise argparse.ArgumentTypeError(message)
    return parsed


def parse_positive(value: str) -> float:
    """解析必须大于零的周期、频率或超时时间。

    Args:
        value: 命令行输入字符串，不允许零。
    Returns:
        转换后的正浮点数。
    Raises:
        ArgumentTypeError: 数值不大于零、为 NaN 或无穷大。
    """
    parsed = float(value)
    if not math.isfinite(parsed) or parsed <= 0.0:
        raise argparse.ArgumentTypeError(f"expected finite value > 0: {value}")
    return parsed


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
        type=parse_non_negative,
        default=20.0,
        help="oscillation duration in seconds; 0 runs until Ctrl+C",
    )
    parser.add_argument(
        "--period",
        type=parse_positive,
        default=4.0,
        help="seconds per complete sine-wave cycle",
    )
    # rate 是服务目标下发频率，不是关节自身控制器的闭环频率。
    parser.add_argument(
        "--rate",
        type=parse_positive,
        default=5.0,
        help="sampled service-command rate in Hz; maximum 20",
    )
    parser.add_argument(
        "--amplitude-deg",
        type=parse_positive,
        default=3.0,
        help="shared arm/head/waist angular amplitude in degrees",
    )
    parser.add_argument(
        "--lumbar-lift-amplitude-mm",
        type=parse_non_negative,
        default=LUMBAR_LIFT_AMPLITUDE_MM,
        help="waist downward travel from zero in mm; maximum 100",
    )
    parser.add_argument(
        "--control-gripper",
        type=parse_switch,
        default=True,
        help="whether both grippers oscillate between 0 and 1",
    )
    parser.add_argument(
        "--work-hold",
        "--zero-hold",
        dest="work_hold",
        type=parse_non_negative,
        default=2.0,
        help="seconds to hold the work pose before oscillation",
    )
    # state-timeout 用于模式/使能确认；service-timeout 用于普通服务响应。
    parser.add_argument(
        "--state-timeout",
        type=parse_positive,
        default=10.0,
        help="seconds to confirm AUTO, IDLE, enable, and disable state",
    )
    parser.add_argument(
        "--service-timeout",
        type=parse_positive,
        default=3.0,
        help="seconds to wait for each sampled service response",
    )
    parser.add_argument(
        "--motion-timeout",
        type=parse_positive,
        default=60.0,
        help="seconds allowed for blocking work/final arm motion",
    )
    # 0 表示使用底层默认值；当前 Nero MoveJ 默认速度为 0.628 rad/s（20%）。
    parser.add_argument(
        "--arm-velocity",
        type=parse_non_negative,
        default=0.0,
        help="MoveJ velocity in rad/s; 0 uses the driver default",
    )
    parser.add_argument(
        "--arm-acceleration",
        type=parse_non_negative,
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


def return_arm_targets(
    robot_name: str,
) -> Tuple[Sequence[float], Sequence[float]]:
    """获取官方 Demo 为指定机器人定义的双臂结束姿态。

    Args:
        robot_name: wheelloong 或 shiloong。
    Returns:
        左右臂各七个关节目标，统一转换为 rad。
    Notes:
        Wheelloong 返回全零；Shiloong 返回上方常量定义的对称姿态。
    """
    if robot_name == "shiloong":
        return (
            tuple(math.radians(value) for value in SHILOONG_RETURN_LEFT_DEG),
            tuple(math.radians(value) for value in SHILOONG_RETURN_RIGHT_DEG),
        )
    return (0.0,) * 7, (0.0,) * 7


def work_arm_targets(
    robot_name: str,
) -> Tuple[Sequence[float], Sequence[float]]:
    """获取指定机器人双臂振荡中心姿态。

    Args:
        robot_name: wheelloong 或 shiloong。
    Returns:
        左右臂各七个工作姿态关节角，单位 rad。
    Notes:
        Wheelloong 暂用全零；Shiloong 使用 WORK 常量。
    """
    if robot_name == "shiloong":
        return (
            tuple(math.radians(value) for value in SHILOONG_WORK_LEFT_DEG),
            tuple(math.radians(value) for value in SHILOONG_WORK_RIGHT_DEG),
        )
    return (0.0,) * 7, (0.0,) * 7


def oscillating_arm_targets(
    robot_name: str, offset_rad: float
) -> Tuple[Sequence[float], Sequence[float]]:
    """把同一正弦偏移叠加到左右臂各自的七轴工作姿态。"""
    left_center, right_center = work_arm_targets(robot_name)
    return (
        tuple(center + offset_rad for center in left_center),
        tuple(center + offset_rad for center in right_center),
    )


def cartesian_pose_6d(
    values: Sequence[float], name: str
) -> CartesianPoseMM:
    """把六维 xyz(mm)+RPY(deg) 转换为四元数笛卡尔位姿。

    Args:
        values: x、y、z、roll、pitch、yaw 六元素序列。
        name: 参数名称，用于生成明确的校验错误。
    Returns:
        使用 arm_driver 坐标系的绝对笛卡尔位姿。
    Raises:
        ValidationError: 元素数量或数值类型不合法。
    """
    try:
        items = tuple(float(value) for value in values)
    except (TypeError, ValueError) as exc:
        raise ValidationError(f"{name} must contain numeric values") from exc
    if len(items) != 6:
        raise ValidationError(f"{name} must contain exactly six values")
    x_mm, y_mm, z_mm, roll_deg, pitch_deg, yaw_deg = items
    return CartesianPoseMM.from_rpy(
        x_mm,
        y_mm,
        z_mm,
        math.radians(roll_deg),
        math.radians(pitch_deg),
        math.radians(yaw_deg),
    )


def offset_pose(
    base: CartesianPoseMM, offset_6d: Sequence[float], name: str
) -> CartesianPoseMM:
    """在当前 TCP 位姿上叠加六维局部姿态偏移。

    Args:
        base: 从 /system/get_info 读取的当前 TCP 位姿。
        offset_6d: dx、dy、dz(mm)和局部 dRPY(deg)。
        name: 左右臂偏移参数名称。
    Returns:
        位置和局部姿态均叠加偏移后的绝对位姿。
    """
    base = base.validated()
    delta = cartesian_pose_6d(offset_6d, name)
    # q_target=q_base*q_delta：姿态增量绕当前 TCP 的局部轴叠加。
    qx = base.qw * delta.qx + base.qx * delta.qw
    qx += base.qy * delta.qz - base.qz * delta.qy
    qy = base.qw * delta.qy - base.qx * delta.qz
    qy += base.qy * delta.qw + base.qz * delta.qx
    qz = base.qw * delta.qz + base.qx * delta.qy
    qz += -base.qy * delta.qx + base.qz * delta.qw
    qw = base.qw * delta.qw - base.qx * delta.qx
    qw += -base.qy * delta.qy - base.qz * delta.qz
    return CartesianPoseMM(
        base.x_mm + delta.x_mm,
        base.y_mm + delta.y_mm,
        base.z_mm + delta.z_mm,
        qx,
        qy,
        qz,
        qw,
        base.frame_id,
    ).validated()


def ensure_auto_ready(robot: Robot, axes: AxisSelection) -> None:
    """在每个振荡采样前检查系统仍适合继续运动。

    Args:
        robot: 已进入 AUTO 会话的机器人对象。
        axes: 本次会话要求保持使能的六组轴。
    Raises:
        RobotStateError: 已退出 AUTO、任一轴失能或系统报告错误。
    Notes:
        状态最大允许缓存 1 秒，过期时最多等待新状态 1 秒。
    """
    state = robot.state(max_age_sec=1.0, wait_timeout_sec=1.0)
    if state.control_mode != int(ControlMode.AUTO):
        raise RobotStateError("System left AUTO mode during oscillation")
    if not axes.satisfied_by(state):
        message = "an actuator lost enable state during oscillation"
        raise RobotStateError(message)
    if state.errors:
        message = "robot reports errors: " + "; ".join(state.errors)
        raise RobotStateError(message)


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
        ensure_auto_ready(robot, axes)
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


def command_move_p_absolute(
    robot: Robot,
    args: argparse.Namespace,
    left_pose_6d: Sequence[float],
    right_pose_6d: Sequence[float],
    speed_rad_s: float = 0.2,
) -> None:
    """使用左右臂六维绝对笛卡尔位姿执行 MoveP。

    Args:
        robot/args: 当前 AUTO 机器人和运行参数。
        left_pose_6d/right_pose_6d: xyz(mm)+RPY(deg)绝对位姿。
        speed_rad_s: 双臂共用的关节速度，默认 0.2 rad/s。
    """
    left_target = cartesian_pose_6d(left_pose_6d, "left_pose_6d")
    right_target = cartesian_pose_6d(right_pose_6d, "right_pose_6d")
    # MoveP 需要当前关节角作为左右臂逆解参考，但目标只来自传入位姿。
    state = robot.state(max_age_sec=0.2, wait_timeout_sec=args.state_timeout)
    print(
        "MoveP absolute left={} right={} speed_rad_s={}".format(
            left_pose_6d, right_pose_6d, speed_rad_s
        )
    )
    robot.arms.move_p(
        left=left_target,
        right=right_target,
        speed_rad_s=speed_rad_s,
        acceleration_rad_s2=0.0,
        left_reference=state.left_arm_joints,
        right_reference=state.right_arm_joints,
        wait=True,
        timeout_sec=args.motion_timeout,
    )


def command_move_l_absolute(
    robot: Robot,
    args: argparse.Namespace,
    left_pose_6d: Sequence[float],
    right_pose_6d: Sequence[float],
    speed_mm_s: float = 10.0,
) -> None:
    """使用左右臂六维绝对笛卡尔位姿执行 MoveL。

    Args:
        robot/args: 当前 AUTO 机器人和运行参数。
        left_pose_6d/right_pose_6d: xyz(mm)+RPY(deg)绝对位姿。
        speed_mm_s: 双臂共用的 TCP 直线速度，默认 10 mm/s。
    """
    left_target = cartesian_pose_6d(left_pose_6d, "left_pose_6d")
    right_target = cartesian_pose_6d(right_pose_6d, "right_pose_6d")
    print(
        "MoveL absolute left={} right={} speed_mm_s={}".format(
            left_pose_6d, right_pose_6d, speed_mm_s
        )
    )
    robot.arms.move_l(
        left=left_target,
        right=right_target,
        speed_mm_s=speed_mm_s,
        acceleration_mm_s2=0.0,
        relative=False,
        left_psi_rad=0.0,
        right_psi_rad=0.0,
        wait=True,
        timeout_sec=args.motion_timeout,
    )


def command_move_p_offset(
    robot: Robot,
    args: argparse.Namespace,
    left_offset_6d: Sequence[float],
    right_offset_6d: Sequence[float],
    speed_rad_s: float = 0.2,
) -> None:
    """读取当前双臂 TCP 并按各自六维偏移执行 MoveP。

    Args:
        robot/args: 当前 AUTO 机器人和运行参数。
        left_offset_6d/right_offset_6d: xyz(mm)+局部RPY(deg)偏移。
        speed_rad_s: 双臂共用的关节速度，默认 0.2 rad/s。
    Raises:
        RobotStateError: 当前双臂四元数位姿无效。
    """

    # 先停止采样，再从 /system/get_info 读取当前 TCP 和逆解参考关节角。
    robot.arms.hold(timeout_sec=args.service_timeout)
    # 等待超过允许缓存年龄，保证下一次读取来自 Hold 之后的新消息。
    time.sleep(0.25)
    state = robot.state(max_age_sec=0.2, wait_timeout_sec=args.state_timeout)
    if state.left_arm_pose is None or state.right_arm_pose is None:
        message = (
            "/system/get_info did not provide valid dual-arm quaternion poses"
        )
        raise RobotStateError(message)
    left_target = offset_pose(
        state.left_arm_pose, left_offset_6d, "left_offset_6d"
    )
    right_target = offset_pose(
        state.right_arm_pose, right_offset_6d, "right_offset_6d"
    )
    print(
        "MoveP current_left={} current_right={}".format(
            state.left_arm_pose, state.right_arm_pose
        )
    )
    print(
        "MoveP offset_left={} offset_right={} speed_rad_s={}".format(
            left_offset_6d,
            right_offset_6d,
            speed_rad_s,
        )
    )
    robot.arms.move_p(
        left=left_target,
        right=right_target,
        speed_rad_s=speed_rad_s,
        acceleration_rad_s2=0.0,
        left_reference=state.left_arm_joints,
        right_reference=state.right_arm_joints,
        wait=True,
        timeout_sec=args.motion_timeout,
    )


def command_move_l_offset(
    robot: Robot,
    args: argparse.Namespace,
    left_offset_6d: Sequence[float],
    right_offset_6d: Sequence[float],
    speed_mm_s: float = 10.0,
) -> None:
    """读取当前双臂 TCP 并按各自六维偏移执行 MoveL。

    Args:
        robot/args: 当前 AUTO 机器人和运行参数。
        left_offset_6d/right_offset_6d: xyz(mm)+局部RPY(deg)偏移。
        speed_mm_s: 双臂共用的 TCP 直线速度，默认 10 mm/s。
    Raises:
        RobotStateError: 当前双臂四元数位姿无效。
    """

    # 明确停止并保持 MoveP 终点，再读取实际到达的当前 TCP 位姿。
    robot.arms.hold(timeout_sec=args.service_timeout)
    # 排除 MoveP 完成前缓存的旧消息，等待 /system/get_info 刷新。
    time.sleep(0.25)
    state = robot.state(max_age_sec=0.2, wait_timeout_sec=args.state_timeout)
    if state.left_arm_pose is None or state.right_arm_pose is None:
        message = (
            "/system/get_info did not provide valid dual-arm quaternion poses"
        )
        raise RobotStateError(message)
    left_target = offset_pose(
        state.left_arm_pose, left_offset_6d, "left_offset_6d"
    )
    right_target = offset_pose(
        state.right_arm_pose, right_offset_6d, "right_offset_6d"
    )
    print(
        "MoveL current_left={} current_right={}".format(
            state.left_arm_pose, state.right_arm_pose
        )
    )
    print(
        "MoveL offset_left={} offset_right={} speed_mm_s={}".format(
            left_offset_6d,
            right_offset_6d,
            speed_mm_s,
        )
    )
    robot.arms.move_l(
        left=left_target,
        right=right_target,
        speed_mm_s=speed_mm_s,
        acceleration_mm_s2=0.0,
        relative=False,
        left_psi_rad=0.0,
        right_psi_rad=0.0,
        wait=True,
        timeout_sec=args.motion_timeout,
    )


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
    left, right = return_arm_targets(args.robot)
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


def cleanup_demo(
    robot: Robot,
    args: argparse.Namespace,
    *,
    auto_requested: bool,
    enabled_requested: bool,
) -> Tuple[BaseException, ...]:
    """使用独立 System 接口尽力完成 Idle 和全部去使能。

    Args:
        robot/args: SDK 对象和状态/服务超时参数。
        auto_requested: 是否已经尝试请求 AUTO，决定是否恢复 Idle。
        enabled_requested: 是否已经尝试上使能，决定是否全部去使能。
    Returns:
        清理期间出现的异常；每一步失败都不会阻止下一步执行。
    """
    errors = []
    if auto_requested:
        print("stage 8/9: switch System to Idle")
        try:
            robot.system.set_control_mode(
                ControlMode.IDLE, timeout_sec=args.service_timeout
            )
            robot.system.wait_for_control_mode(
                ControlMode.IDLE, timeout_sec=args.state_timeout
            )
        except BaseException as exc:
            errors.append(exc)

    if enabled_requested:
        print("stage 9/9: disable both arms, head, and waist")
        try:
            robot.system.disable_all(timeout_sec=args.service_timeout)
            robot.system.wait_for_all_disabled(timeout_sec=args.state_timeout)
        except BaseException as exc:
            errors.append(exc)
    return tuple(errors)


def run_demo(robot: Robot, args: argparse.Namespace) -> None:
    """执行九阶段 AUTO 振荡及 MoveP/MoveL 测试。

    Args:
        robot: 已创建但尚未进入 AUTO 的机器人对象。
        args: 已校验的测试参数。
    Notes:
        每个阶段显式调用独立公共接口，便于与官方 Demo 逐项对照。
        finally 保证正常结束或抛出异常时都会尝试 Hold、Idle 和去使能。
    """
    # 官方测试同时使用双臂、头部两轴和腰部两轴，因此选择全部六组。
    axes = AxisSelection.all()
    auto_requested = False
    enabled_requested = False
    motion_started = False
    final_pose_completed = False
    try:
        print("stage 1/9: switch System to AUTO")
        # 设置请求和状态确认是两个独立接口，与官方 set/wait 步骤一致。
        auto_requested = True
        robot.system.set_control_mode(
            ControlMode.AUTO, timeout_sec=args.service_timeout
        )
        robot.system.wait_for_control_mode(
            ControlMode.AUTO, timeout_sec=args.state_timeout
        )

        print("stage 2/9: enable both arms, head, and waist")
        enabled_requested = True
        robot.system.set_enabled(axes, timeout_sec=args.service_timeout)
        robot.system.wait_for_enabled(axes, timeout_sec=args.state_timeout)

        print("stage 3/9: move arms to work pose and head/waist to zero")
        # 先停止遗留运动，再让双臂进入振荡中心，头腰和夹爪回零。
        robot.arms.hold(timeout_sec=args.service_timeout)
        motion_started = True
        work_left, work_right = work_arm_targets(args.robot)
        command_test_pose(
            robot=robot,
            args=args,
            left=work_left,
            right=work_right,
            body_targets=[
                0.0,  # 头部俯仰，rad
                0.0,  # 头部旋转，rad
                0.0,  # 腰部俯仰，rad
                0.0,  # 腰部升降，mm
            ],
            gripper_position=0.0,
            blocking=True,
        )
        if args.work_hold > 0.0:
            # 给工作姿态留出稳定时间，再开始连续非阻塞采样。
            time.sleep(args.work_hold)

        print("stage 4/9: oscillate arms, head, and waist; chassis untouched")
        oscillation_interrupted = False
        try:
            run_oscillation_samples(robot, args, axes)
        except KeyboardInterrupt:
            # 振荡阶段 Ctrl+C 被视为“提前结束采样”，仍执行结束姿态和清理。
            oscillation_interrupted = True
            print(
                "oscillation interrupted; skip MoveP/MoveL and use final pose"
            )

        if not oscillation_interrupted:
            print("stage 5/9: stop sampling and run MoveP offset command")
            command_move_p_offset(
                robot,
                args,
                left_offset_6d=(0.0, 20.0, 30.0, 0.0, 0.0, 0.0),
                right_offset_6d=(0.0, -20.0, 30.0, 0.0, 0.0, 0.0),
                speed_rad_s=0.2,
            )

            print("stage 6/9: Hold, read TCP poses, then run MoveL offset")
            command_move_l_offset(
                robot,
                args,
                left_offset_6d=(0.0, -20.0, -30.0, 0.0, 0.0, 0.0),
                right_offset_6d=(0.0, 20.0, -30.0, 0.0, 0.0, 0.0),
            )
        else:
            print("stage 5/9: MoveP offset skipped after Ctrl+C")
            print("stage 6/9: MoveL offset skipped after Ctrl+C")

        print("stage 7/9: Hold, then move to robot-specific final pose")
        command_final_pose(robot, args)
        final_pose_completed = True
    finally:
        # 保存是否已有业务异常；清理异常不能覆盖更早、更有价值的异常。
        active_exception = sys.exc_info()[0] is not None
        cleanup_errors = []
        if motion_started and not final_pose_completed:
            try:
                robot.arms.hold(timeout_sec=args.service_timeout)
            except BaseException as exc:
                cleanup_errors.append(exc)
        cleanup_errors.extend(
            cleanup_demo(
                robot,
                args,
                auto_requested=auto_requested,
                enabled_requested=enabled_requested,
            )
        )
        if cleanup_errors:
            details = "; ".join(str(error) for error in cleanup_errors)
            if active_exception:
                print(f"cleanup also failed: {details}", file=sys.stderr)
            else:
                raise cleanup_errors[0]


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
        print("demo interrupted before oscillation cleanup", file=sys.stderr)
    except WheelloongSdkError as exc:
        print(f"AUTO oscillation demo failed: {exc}", file=sys.stderr)
    return 1


if __name__ == "__main__":
    sys.exit(main())
