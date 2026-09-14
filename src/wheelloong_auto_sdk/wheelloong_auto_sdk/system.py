"""Public System mode, state, and actuator-enable API."""

from .backend.base import RobotBackend
from .errors import RobotStateError, ValidationError
from .models import (
    AxisSelection,
    CommandResult,
    ControlMode,
    SystemState,
    finite_float,
)


class System:
    """提供系统状态、AUTO/IDLE 模式及六组轴使能控制接口。"""

    def __init__(self, backend: RobotBackend) -> None:
        """使用指定机器人后端创建系统控制器。"""
        self._backend = backend

    @staticmethod
    def _positive(value: float, name: str) -> float:
        """校验超时时间为有限正数。"""
        result = finite_float(value, name)
        if result <= 0.0:
            raise ValidationError(f"{name} must be greater than zero")
        return result

    @staticmethod
    def _mode(value: ControlMode) -> ControlMode:
        """将输入校验并转换为 SDK 允许的 AUTO 或 IDLE 模式。"""
        try:
            return ControlMode(value)
        except (TypeError, ValueError) as exc:
            raise ValidationError("mode must be ControlMode.AUTO or IDLE") from exc

    @staticmethod
    def _axes(value: AxisSelection) -> AxisSelection:
        """校验轴使能目标使用 AxisSelection 表达。"""
        if not isinstance(value, AxisSelection):
            raise ValidationError("axes must be an AxisSelection")
        return value

    @staticmethod
    def _enabled_exactly(state: SystemState, axes: AxisSelection) -> bool:
        """判断系统六组轴的实际使能值是否与目标完全一致。"""
        return (
            state.left_arm_enabled == axes.left_arm
            and state.right_arm_enabled == axes.right_arm
            and state.head_pitch_enabled == axes.head_pitch
            and state.head_yaw_enabled == axes.head_yaw
            and state.waist_pitch_enabled == axes.waist_pitch
            and state.waist_lift_enabled == axes.waist_lift
        )

    def state(
        self, *, max_age_sec: float = 0.5, wait_timeout_sec: float = 2.0
    ) -> SystemState:
        """获取满足新鲜度要求的系统状态。

        Args:
            max_age_sec: 可接受状态缓存的最大年龄。
            wait_timeout_sec: 等待新状态到达的最长秒数。
        Returns:
            不包含 ROS 消息类型的系统状态快照。
        Raises:
            ValidationError: 任一时间参数不大于零。
        """
        max_age = self._positive(max_age_sec, "max_age_sec")
        wait_timeout = self._positive(wait_timeout_sec, "wait_timeout_sec")
        return self._backend.get_system_state(max_age, wait_timeout)

    def set_control_mode(
        self, mode: ControlMode, *, timeout_sec: float = 3.0
    ) -> CommandResult:
        """独立请求切换 AUTO 或 IDLE 控制模式。

        Args:
            mode: ControlMode.AUTO 或 ControlMode.IDLE。
            timeout_sec: 等待模式服务响应的最长秒数。
        Returns:
            模式服务成功响应结果；本方法不等待状态话题确认。
        Raises:
            ValidationError: 模式或超时时间不合法。
        """
        selected = self._mode(mode)
        timeout = self._positive(timeout_sec, "timeout_sec")
        return self._backend.set_control_mode(int(selected), timeout)

    def wait_for_control_mode(
        self, mode: ControlMode, *, timeout_sec: float = 10.0
    ) -> SystemState:
        """独立等待状态话题确认指定控制模式。

        Args:
            mode: 期望确认的 AUTO 或 IDLE 模式。
            timeout_sec: 等待 SystemInfo 状态满足条件的最长秒数。
        Returns:
            已确认达到目标模式的系统状态。
        Raises:
            CommandTimeoutError: 截止时间前未确认目标模式。
        """
        selected = self._mode(mode)
        timeout = self._positive(timeout_sec, "timeout_sec")
        return self._backend.wait_system_state(
            lambda state: state.control_mode == int(selected),
            timeout,
            f"{selected.name} control mode",
        )

    def set_enabled(
        self, axes: AxisSelection, *, timeout_sec: float = 3.0
    ) -> CommandResult:
        """独立设置六组手臂、头部和腰部轴的使能值。

        Args:
            axes: 六组轴的完整目标；False 表示对应轴去使能。
            timeout_sec: 等待使能服务响应的最长秒数。
        Returns:
            使能服务成功响应结果；本方法不等待状态话题确认。
        Raises:
            ValidationError: 轴配置或超时时间不合法。
        """
        selected = self._axes(axes)
        timeout = self._positive(timeout_sec, "timeout_sec")
        return self._backend.set_enabled(selected, timeout)

    def wait_for_enabled(
        self, axes: AxisSelection, *, timeout_sec: float = 10.0
    ) -> SystemState:
        """独立等待六组轴的使能状态与目标完全一致。

        Args:
            axes: 期望确认的六组轴完整使能状态。
            timeout_sec: 等待 SystemInfo 状态满足条件的最长秒数。
        Returns:
            已确认使能状态与目标完全一致的系统状态。
        Raises:
            CommandTimeoutError: 截止时间前未确认目标状态。
        """
        selected = self._axes(axes)
        timeout = self._positive(timeout_sec, "timeout_sec")
        return self._backend.wait_system_state(
            lambda state: self._enabled_exactly(state, selected),
            timeout,
            "requested arm/head/waist enable state",
        )

    def enable_all(self, *, timeout_sec: float = 3.0) -> CommandResult:
        """请求使能双臂、头部两轴和腰部两轴。"""
        return self.set_enabled(AxisSelection.all(), timeout_sec=timeout_sec)

    def disable_all(self, *, timeout_sec: float = 3.0) -> CommandResult:
        """请求去使能双臂、头部两轴和腰部两轴。"""
        return self.set_enabled(AxisSelection.none(), timeout_sec=timeout_sec)

    def wait_for_all_disabled(self, *, timeout_sec: float = 10.0) -> SystemState:
        """等待状态话题确认双臂、头部和腰部已经全部去使能。"""
        return self.wait_for_enabled(
            AxisSelection.none(), timeout_sec=timeout_sec
        )

    def require_ready(
        self,
        required_axes: AxisSelection,
        *,
        max_age_sec: float = 1.0,
        wait_timeout_sec: float = 1.0,
    ) -> SystemState:
        """确认系统处于 AUTO、指定轴仍使能且不存在活动错误。

        Args:
            required_axes: 当前任务必须保持使能的轴。
            max_age_sec: 可接受状态缓存的最大年龄。
            wait_timeout_sec: 等待新状态的最长秒数。
        Returns:
            已通过三项检查的状态快照。
        Raises:
            RobotStateError: 模式、使能或错误状态不满足运动条件。
            ValidationError: 轴选择或时间参数不合法。
        """
        axes = self._axes(required_axes)
        state = self.state(
            max_age_sec=max_age_sec,
            wait_timeout_sec=wait_timeout_sec,
        )
        if state.control_mode != int(ControlMode.AUTO):
            raise RobotStateError("System is not in AUTO mode")
        if not axes.satisfied_by(state):
            raise RobotStateError("a required actuator is not enabled")
        if state.errors:
            raise RobotStateError(
                "robot reports errors: " + "; ".join(state.errors)
            )
        return state
