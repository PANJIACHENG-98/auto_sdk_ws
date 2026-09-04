#!/usr/bin/env python3
"""用两个独立函数演示不包含目标运动的 AUTO 生命周期。

`enter_auto_stage()` 显式完成状态读取、AUTO、使能和 Hold；
`exit_auto_stage()` 显式完成 Hold、IDLE、去使能和退出状态打印。
程序不调用 MoveJ、头腰、夹爪、导航或语音接口。
"""

import argparse
import math
import sys
import time
from typing import Callable, List, Optional, TypeVar

from wheelloong_auto_sdk import (
    AxisSelection,
    BackendError,
    CommandTimeoutError,
    ControlMode,
    Robot,
    RobotCommandError,
    RobotStateError,
    ServiceUnavailableError,
    SystemState,
    ValidationError,
    WheelloongSdkError,
)


ResultT = TypeVar("ResultT")


class DemoStepError(WheelloongSdkError):
    """表示 AUTO 生命周期样例中一个已命名步骤失败。"""

    def __init__(self, stage: str, step: str, cause: Exception) -> None:
        """保存失败阶段、步骤和原始异常，并生成可操作的信息。"""
        self.stage = stage
        self.step = step
        self.original_error = cause
        super().__init__(f"{stage} / {step}：{describe_failure(cause)}")


class DemoCleanupError(WheelloongSdkError):
    """表示阶段二尽力清理后仍有一个或多个步骤失败。"""

    def __init__(self, errors: List[DemoStepError]) -> None:
        """汇总全部清理错误，避免只显示第一个失败。"""
        self.errors = tuple(errors)
        details = "；".join(str(error) for error in errors)
        super().__init__(f"阶段二共有 {len(errors)} 个步骤失败：{details}")


def describe_failure(error: Exception) -> str:
    """根据 SDK 异常类型生成不同原因及排查方向。

    Args:
        error: SDK 或 Python 操作抛出的原始异常。
    Returns:
        包含错误类别、底层细节和建议的中文说明。
    """
    if isinstance(error, RobotCommandError):
        detail = error.robot_message.strip() or "底层未返回详细信息"
        return (
            "机器人拒绝命令"
            f"（operation={error.operation}, error_code={error.error_code}）：{detail}"
        )
    if isinstance(error, ServiceUnavailableError):
        return (
            "所需 ROS 服务或动作不可用；请确认机器人系统已启动，且当前终端的 "
            f"ROS 通信环境与机器人进程一致。底层信息：{error}"
        )
    if isinstance(error, CommandTimeoutError):
        return f"等待超时；命令可能未生效或状态话题未更新。底层信息：{error}"
    if isinstance(error, RobotStateError):
        return f"机器人状态不可用、过期或不满足前置条件。底层信息：{error}"
    if isinstance(error, ValidationError):
        return f"输入参数不合法。底层信息：{error}"
    if isinstance(error, BackendError):
        return f"ROS 后端通信失败。底层信息：{error}"
    if isinstance(error, WheelloongSdkError):
        return f"SDK 执行失败。底层信息：{error}"
    return f"未预期的 {type(error).__name__}：{error}"


def run_step(
    stage: str, step: str, operation: Callable[[], ResultT]
) -> ResultT:
    """执行一个已命名步骤，并为异常补充阶段及原因。

    Args:
        stage: 阶段一、阶段二或观察阶段。
        step: 当前具体操作名称。
        operation: 不接收参数的实际操作。
    Returns:
        实际操作的返回值。
    Raises:
        KeyboardInterrupt: 用户在该步骤按下 Ctrl+C。
        DemoStepError: 操作因其他原因失败。
    """
    try:
        return operation()
    except KeyboardInterrupt:
        print(
            f"[用户中断] {stage} / {step}：收到 Ctrl+C",
            file=sys.stderr,
        )
        raise
    except Exception as error:
        raise DemoStepError(stage, step, error) from error


def try_cleanup_step(
    step: str,
    operation: Callable[[], ResultT],
    errors: List[DemoStepError],
) -> Optional[ResultT]:
    """执行阶段二步骤，失败时记录并继续后续安全清理。"""
    try:
        return run_step("阶段二", step, operation)
    except DemoStepError as error:
        errors.append(error)
        print(f"[清理步骤失败] {error}", file=sys.stderr)
        return None


def parse_non_negative(value: str) -> float:
    """解析允许为零的有限秒数。

    Args:
        value: 命令行输入的时间字符串。
    Returns:
        转换后的非负浮点秒数。
    Raises:
        ArgumentTypeError: 数值为负数、NaN 或无穷大。
    """
    parsed = float(value)
    if not math.isfinite(parsed) or parsed < 0.0:
        raise argparse.ArgumentTypeError(f"expected finite value >= 0: {value}")
    return parsed


def parse_positive(value: str) -> float:
    """解析必须大于零的有限超时时间。

    Args:
        value: 命令行输入的时间字符串。
    Returns:
        转换后的正浮点秒数。
    Raises:
        ArgumentTypeError: 数值不大于零、为 NaN 或无穷大。
    """
    parsed = float(value)
    if not math.isfinite(parsed) or parsed <= 0.0:
        raise argparse.ArgumentTypeError(f"expected finite value > 0: {value}")
    return parsed


def build_parser() -> argparse.ArgumentParser:
    """创建纯 AUTO 生命周期样例的命令行解析器。

    Returns:
        包含观察时间以及状态、服务超时参数的解析器。
    Notes:
        观察期间不会发送运动目标，只保持 AUTO 和全部轴使能。
    """
    parser = argparse.ArgumentParser(
        description="Enter and leave auto_session without target motion.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--observe-seconds",
        type=parse_non_negative,
        default=2.0,
        help="seconds to remain inside AUTO before normal cleanup",
    )
    parser.add_argument(
        "--state-timeout",
        type=parse_positive,
        default=10.0,
        help="seconds to confirm AUTO/IDLE and enable/disable state",
    )
    parser.add_argument(
        "--service-timeout",
        type=parse_positive,
        default=3.0,
        help="seconds to wait for each System or Hold service response",
    )
    return parser


def enabled_summary(state: SystemState) -> str:
    """将六组轴的使能状态格式化为一行可读文本。

    Args:
        state: SDK 返回的系统状态快照。
    Returns:
        按左臂、右臂、头部两轴和腰部两轴排列的文本。
    """
    return (
        f"left_arm={state.left_arm_enabled}, "
        f"right_arm={state.right_arm_enabled}, "
        f"head_pitch={state.head_pitch_enabled}, "
        f"head_yaw={state.head_yaw_enabled}, "
        f"waist_pitch={state.waist_pitch_enabled}, "
        f"waist_lift={state.waist_lift_enabled}"
    )


def print_state(label: str, state: SystemState) -> None:
    """打印一个阶段的控制模式、使能状态及状态年龄。

    Args:
        label: before、inside 或 after 等阶段名称。
        state: 对应阶段取得的新鲜系统状态。
    """
    print(
        f"[{label}] mode={state.control_mode}, "
        f"enabled=({enabled_summary(state)}), age={state.age_sec:.3f}s"
    )


def enter_auto_stage(robot: Robot, args: argparse.Namespace) -> None:
    """执行阶段一：读取状态、进入 AUTO、全部上使能并 Hold。

    Args:
        robot: 已连接 ROS、尚未进入 AUTO 的机器人对象。
        args: 状态确认和服务响应超时参数。
    Raises:
        WheelloongSdkError: 任一步请求或状态确认失败。
    """
    axes = AxisSelection.all()

    print("stage 1: read state, enter AUTO, enable all, and Hold")
    # 1. 只读取初始状态，不在这里隐式改变任何模式或使能值。
    before = run_step(
        "阶段一",
        "读取初始系统状态",
        lambda: robot.system.state(wait_timeout_sec=args.state_timeout),
    )
    print_state("before enter_auto_stage", before)

    # 2. 请求 AUTO；set 只等服务响应，wait 独立确认实际模式。
    run_step(
        "阶段一",
        "请求切换到 AUTO",
        lambda: robot.system.set_control_mode(
            ControlMode.AUTO, timeout_sec=args.service_timeout
        ),
    )
    run_step(
        "阶段一",
        "确认实际模式为 AUTO",
        lambda: robot.system.wait_for_control_mode(
            ControlMode.AUTO, timeout_sec=args.state_timeout
        ),
    )

    # 3. 请求六组轴全部上使能，再从 SystemInfo 确认每一组实际状态。
    run_step(
        "阶段一",
        "请求双臂、头部和腰部全部上使能",
        lambda: robot.system.set_enabled(
            axes, timeout_sec=args.service_timeout
        ),
    )
    run_step(
        "阶段一",
        "确认双臂、头部和腰部全部已使能",
        lambda: robot.system.wait_for_enabled(
            axes, timeout_sec=args.state_timeout
        ),
    )

    # 4. 停止双臂可能遗留的轨迹并保持当前位置；不会生成新目标位姿。
    run_step(
        "阶段一",
        "双臂 Hold",
        lambda: robot.arms.hold(timeout_sec=args.service_timeout),
    )
    inside = run_step(
        "阶段一",
        "读取进入后的系统状态",
        lambda: robot.system.state(wait_timeout_sec=args.state_timeout),
    )
    print_state("after enter_auto_stage", inside)


def exit_auto_stage(robot: Robot, args: argparse.Namespace) -> None:
    """执行阶段二：Hold、进入 IDLE、全部去使能并打印状态。

    Args:
        robot: 当前可能处于 AUTO 和使能状态的机器人对象。
        args: 状态确认和服务响应超时参数。
    Raises:
        BaseException: 全部清理步骤执行后重新抛出第一个失败。
    """
    errors: List[DemoStepError] = []
    print("stage 2: Hold, enter IDLE, disable all, and read final state")

    # 1. Hold 服务要求双臂均已使能；先读取状态以避免无效调用。
    before_exit = None
    try:
        before_exit = run_step(
            "阶段二",
            "读取退出前状态以判断是否允许 Hold",
            lambda: robot.system.state(wait_timeout_sec=args.state_timeout),
        )
    except DemoStepError as error:
        print(
            f"[Hold 前置检查警告] {error}；无法判断双臂使能状态，仍尝试 Hold",
            file=sys.stderr,
        )

    if before_exit is None or (
        before_exit.left_arm_enabled and before_exit.right_arm_enabled
    ):
        try_cleanup_step(
            "双臂 Hold",
            lambda: robot.arms.hold(timeout_sec=args.service_timeout),
            errors,
        )
    else:
        print(
            "[跳过 Hold] Hold 要求双臂均已使能；"
            f"当前 left_arm={before_exit.left_arm_enabled}, "
            f"right_arm={before_exit.right_arm_enabled}。"
        )

    # 2. 请求 IDLE，并独立等待 SystemInfo 确认实际模式。
    try_cleanup_step(
        "请求切换到 IDLE",
        lambda: robot.system.set_control_mode(
            ControlMode.IDLE, timeout_sec=args.service_timeout
        ),
        errors,
    )
    try_cleanup_step(
        "确认实际模式为 IDLE",
        lambda: robot.system.wait_for_control_mode(
            ControlMode.IDLE, timeout_sec=args.state_timeout
        ),
        errors,
    )

    # 3. 无论前一步是否成功，都尝试六组轴全部去使能并确认。
    try_cleanup_step(
        "请求双臂、头部和腰部全部去使能",
        lambda: robot.system.disable_all(timeout_sec=args.service_timeout),
        errors,
    )
    try_cleanup_step(
        "确认双臂、头部和腰部全部已去使能",
        lambda: robot.system.wait_for_all_disabled(
            timeout_sec=args.state_timeout
        ),
        errors,
    )

    # 4. 最后读取并打印退出后的实际模式、使能状态和状态年龄。
    after = try_cleanup_step(
        "读取退出后的最终状态",
        lambda: robot.system.state(wait_timeout_sec=args.state_timeout),
        errors,
    )
    if after is not None:
        print_state("after exit_auto_stage", after)

    if errors:
        raise DemoCleanupError(errors)


def run_demo(robot: Robot, args: argparse.Namespace) -> None:
    """按顺序调用两个独立阶段，并保证异常时仍执行阶段二。

    Args:
        robot: 已连接 ROS 的机器人对象。
        args: 观察时间以及状态、服务超时参数。
    Notes:
        阶段一成功后只观察，不发送任何目标运动命令。
    """
    try:
        enter_auto_stage(robot, args)
        print("no target motion command will be sent")
        if args.observe_seconds > 0.0:
            run_step(
                "观察阶段",
                f"保持 AUTO 状态 {args.observe_seconds:g} 秒",
                lambda: time.sleep(args.observe_seconds),
            )
    except BaseException:
        # 保留阶段一或 Ctrl+C 的原始异常；阶段二失败只额外打印。
        try:
            exit_auto_stage(robot, args)
        except KeyboardInterrupt:
            print(
                "[清理被中断] 阶段二再次收到 Ctrl+C，安全状态未完全确认",
                file=sys.stderr,
            )
        except Exception as cleanup_error:
            print(
                f"[异常后的清理未完整完成] {cleanup_error}",
                file=sys.stderr,
            )
        else:
            print("[异常后的清理完成] 已执行 IDLE、去使能并读取最终状态")
        raise
    else:
        exit_auto_stage(robot, args)


def main() -> int:
    """创建独立 SDK 运行时并返回样例进程退出码。

    Returns:
        0 表示进入、退出和清理全部成功；1 表示中断或 SDK 异常。
    Notes:
        外层 Robot 上下文负责释放 SDK 自己创建的 ROS 资源。
    """
    args = build_parser().parse_args()
    try:
        with Robot.standalone(node_name="sdk_auto_session_demo") as robot:
            run_demo(robot, args)
        print("two-stage AUTO lifecycle completed successfully")
        return 0
    except KeyboardInterrupt:
        print("AUTO session demo 由用户中断；请核对上方清理结果", file=sys.stderr)
    except DemoStepError as exc:
        print(f"AUTO session demo 步骤失败：{exc}", file=sys.stderr)
    except DemoCleanupError as exc:
        print(f"AUTO session demo 清理未完整完成：{exc}", file=sys.stderr)
    except WheelloongSdkError as exc:
        print(f"AUTO session demo SDK 失败：{describe_failure(exc)}", file=sys.stderr)
    except Exception as exc:
        print(f"AUTO session demo 程序异常：{describe_failure(exc)}", file=sys.stderr)
    return 1


if __name__ == "__main__":
    sys.exit(main())
