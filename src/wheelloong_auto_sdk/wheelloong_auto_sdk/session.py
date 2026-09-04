"""AUTO-mode lifecycle context manager."""

from typing import List, Optional, TYPE_CHECKING

from .errors import RobotStateError
from .models import AxisSelection, ControlMode

if TYPE_CHECKING:
    from .robot import Robot


class AutoSession:
    """安全进入 AUTO、确认轴使能，并在所有退出路径执行清理。"""

    def __init__(
        self,
        robot: "Robot",
        required_axes: AxisSelection,
        state_timeout_sec: float,
        service_timeout_sec: float,
        max_state_age_sec: float,
    ) -> None:
        """保存会话所需机器人、轴配置和状态/服务超时参数。"""
        self._robot = robot
        self._backend = robot._backend
        self._required_axes = required_axes
        self._state_timeout_sec = state_timeout_sec
        self._service_timeout_sec = service_timeout_sec
        self._max_state_age_sec = max_state_age_sec
        self._auto_requested = False
        self._enable_requested = False

    def __enter__(self) -> "Robot":
        """检查状态、切换 AUTO、使能所需轴并 Hold 手臂。"""
        try:
            state = self._robot.system.state(
                max_age_sec=self._max_state_age_sec,
                wait_timeout_sec=self._state_timeout_sec,
            )
            if state.errors:
                raise RobotStateError(
                    "robot reports active errors before AUTO entry: " + "; ".join(state.errors)
                )

            self._auto_requested = True
            self._robot.system.set_control_mode(
                ControlMode.AUTO, timeout_sec=self._service_timeout_sec
            )
            self._robot.system.wait_for_control_mode(
                ControlMode.AUTO, timeout_sec=self._state_timeout_sec
            )

            if not self._required_axes.is_empty:
                self._enable_requested = True
                self._robot.system.set_enabled(
                    self._required_axes, timeout_sec=self._service_timeout_sec
                )
                self._robot.system.wait_for_enabled(
                    self._required_axes, timeout_sec=self._state_timeout_sec
                )

            if self._required_axes.left_arm or self._required_axes.right_arm:
                self._backend.arm_hold(self._service_timeout_sec)
            return self._robot
        except Exception:
            self._cleanup(suppress=True)
            raise

    def __exit__(self, exc_type, exc, traceback) -> bool:
        """执行导航取消、Hold、Idle 和去使能且保留业务异常。"""
        cleanup_error = self._cleanup(suppress=exc is not None)
        if exc is None and cleanup_error is not None:
            raise cleanup_error
        return False

    def _cleanup(self, suppress: bool) -> Optional[BaseException]:
        """尽力执行全部退出步骤并返回第一个清理异常。"""
        errors: List[BaseException] = []
        try:
            self._robot.navigation.cancel_all(timeout_sec=self._service_timeout_sec)
        except BaseException as exc:
            errors.append(exc)
        if self._required_axes.left_arm or self._required_axes.right_arm:
            try:
                self._backend.arm_hold(self._service_timeout_sec)
            except BaseException as exc:
                errors.append(exc)
        if self._auto_requested:
            try:
                self._robot.system.set_control_mode(
                    ControlMode.IDLE, timeout_sec=self._service_timeout_sec
                )
                self._robot.system.wait_for_control_mode(
                    ControlMode.IDLE, timeout_sec=self._state_timeout_sec
                )
            except BaseException as exc:
                errors.append(exc)
        if self._enable_requested:
            try:
                self._robot.system.disable_all(
                    timeout_sec=self._service_timeout_sec
                )
                self._robot.system.wait_for_all_disabled(
                    timeout_sec=self._state_timeout_sec
                )
            except BaseException as exc:
                errors.append(exc)
        self._auto_requested = False
        self._enable_requested = False
        if errors and suppress:
            logger = getattr(self._backend.node, "get_logger", lambda: None)()
            if logger is not None:
                logger.error("AUTO cleanup error: " + "; ".join(str(item) for item in errors))
        return errors[0] if errors else None
