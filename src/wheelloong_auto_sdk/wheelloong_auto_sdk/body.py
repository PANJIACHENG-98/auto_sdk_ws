"""Public head and waist control API."""

from typing import Optional

from .backend.base import RobotBackend
from .errors import ValidationError
from .models import CommandResult, finite_float


class Body:
    """提供头部俯仰/转动及腰部俯仰/升降控制接口。"""

    def __init__(self, backend: RobotBackend) -> None:
        """使用指定机器人后端创建头腰控制器。"""
        self._backend = backend

    def move(
        self,
        *,
        head_pitch_rad: Optional[float] = None,
        head_yaw_rad: Optional[float] = None,
        waist_pitch_rad: Optional[float] = None,
        waist_lift_mm: Optional[float] = None,
        speed_percent: float = 0.0,
        wait: bool = True,
        timeout_sec: float = 10.0,
    ) -> CommandResult:
        """同时控制明确给出的头部和腰部轴，未给出的轴保持不受控。

        Args:
            head_pitch_rad/head_yaw_rad: 可选头部俯仰和转动目标，单位 rad。
            waist_pitch_rad: 可选腰部俯仰目标，单位 rad。
            waist_lift_mm: 可选腰部升降目标，单位 mm；向下通常为负值。
            speed_percent: 预留速度百分比；0 使用驱动默认值，范围 [0, 100]。
            wait: 是否阻塞等待；timeout_sec 范围为 (0, 30] 秒。
        Returns:
            成功指令结果。
        Raises:
            ValidationError: 未指定轴或参数范围不合法。
        """
        values = (head_pitch_rad, head_yaw_rad, waist_pitch_rad, waist_lift_mm)
        if all(value is None for value in values):
            raise ValidationError("at least one head or waist target must be supplied")
        converted = tuple(
            None if value is None else finite_float(value, name)
            for value, name in zip(
                values,
                ("head_pitch_rad", "head_yaw_rad", "waist_pitch_rad", "waist_lift_mm"),
            )
        )
        speed = finite_float(speed_percent, "speed_percent")
        if not 0.0 <= speed <= 100.0:
            raise ValidationError("speed_percent must be in [0, 100]")
        timeout = finite_float(timeout_sec, "timeout_sec")
        # JntsCtrl 接口规定 timeout_sec 始终写入 (0, 30]，即使当前非阻塞时未使用。
        if timeout <= 0.0 or timeout > 30.0:
            raise ValidationError("timeout_sec must be in (0, 30] for body control")
        return self._backend.move_body(
            converted[0],
            converted[1],
            converted[2],
            converted[3],
            speed,
            bool(wait),
            timeout,
        )

    def move_head(
        self,
        *,
        pitch_rad: Optional[float] = None,
        yaw_rad: Optional[float] = None,
        speed_percent: float = 0.0,
        wait: bool = True,
        timeout_sec: float = 10.0,
    ) -> CommandResult:
        """控制头部俯仰、转动或两轴组合运动。"""
        return self.move(
            head_pitch_rad=pitch_rad,
            head_yaw_rad=yaw_rad,
            speed_percent=speed_percent,
            wait=wait,
            timeout_sec=timeout_sec,
        )

    def move_waist(
        self,
        *,
        pitch_rad: Optional[float] = None,
        lift_mm: Optional[float] = None,
        speed_percent: float = 0.0,
        wait: bool = True,
        timeout_sec: float = 10.0,
    ) -> CommandResult:
        """控制腰部俯仰、升降或两轴组合运动。"""
        return self.move(
            waist_pitch_rad=pitch_rad,
            waist_lift_mm=lift_mm,
            speed_percent=speed_percent,
            wait=wait,
            timeout_sec=timeout_sec,
        )
