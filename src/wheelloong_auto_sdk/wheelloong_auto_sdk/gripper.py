"""Public gripper API."""

from .backend.base import RobotBackend
from .errors import ValidationError
from .models import ArmSide, CommandResult, finite_float


class Gripper:
    """提供左、右或双夹爪的位置控制和状态确认接口。"""

    def __init__(self, backend: RobotBackend) -> None:
        """使用指定机器人后端创建夹爪控制器。"""
        self._backend = backend

    def set(
        self,
        side: ArmSide,
        position: float,
        *,
        speed: float = 0.0,
        torque: float = 0.0,
        wait: bool = True,
        tolerance: float = 0.02,
        timeout_sec: float = 10.0,
    ) -> CommandResult:
        """设置一个或两个夹爪的目标行程比例。

        Args:
            side: LEFT、RIGHT 或 DUAL 夹爪选择。
            position: 官方协议目标比例，范围 [0, 1]；0 张开、1 闭合。
            speed/torque: 最大速度和力矩比例；小于等于零使用默认值。
            wait: 是否等待实际夹爪状态达到目标。
            tolerance: 状态确认允许误差；timeout_sec 为最长等待秒数。
        Returns:
            成功指令结果。
        Raises:
            ValidationError: 选择、比例、误差或超时时间不合法。
        """
        try:
            selected = ArmSide(side)
        except (TypeError, ValueError) as exc:
            raise ValidationError("side must be ArmSide.LEFT, RIGHT, or DUAL") from exc
        target = finite_float(position, "position")
        speed_value = finite_float(speed, "speed")
        torque_value = finite_float(torque, "torque")
        tolerance_value = finite_float(tolerance, "tolerance")
        timeout = finite_float(timeout_sec, "timeout_sec")
        if not 0.0 <= target <= 1.0:
            raise ValidationError("position must be in [0, 1]")
        if speed_value > 1.0 or torque_value > 1.0:
            raise ValidationError("speed and torque must be <= 1; values <= 0 use defaults")
        if tolerance_value < 0.0 or timeout <= 0.0:
            raise ValidationError("tolerance must be >= 0 and timeout_sec must be > 0")
        return self._backend.set_gripper(
            int(selected),
            target,
            speed_value,
            torque_value,
            bool(wait),
            tolerance_value,
            timeout,
        )

    def open(self, side: ArmSide, **kwargs) -> CommandResult:
        """将指定夹爪张开到官方协议位置 0.0。"""
        return self.set(side, 0.0, **kwargs)

    def close(self, side: ArmSide, **kwargs) -> CommandResult:
        """将指定夹爪闭合到官方协议位置 1.0。"""
        return self.set(side, 1.0, **kwargs)
