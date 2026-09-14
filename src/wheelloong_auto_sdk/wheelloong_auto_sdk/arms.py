"""Public dual-arm motion API."""

import time
from typing import Optional, Sequence, Tuple

from .backend.base import RobotBackend
from .errors import CommandTimeoutError, RobotStateError, ValidationError
from .models import (
    ARM_DOF,
    CartesianPoseMM,
    CommandResult,
    MotionMode,
    SystemState,
    finite_float,
    finite_tuple,
)


class Arms:
    """提供双臂关节、直线、点到点运动以及急停保持接口。"""

    def __init__(self, backend: RobotBackend) -> None:
        """使用指定机器人后端创建双臂控制器。"""
        self._backend = backend

    @staticmethod
    def _joints(values: Optional[Sequence[float]], name: str) -> Optional[Tuple[float, ...]]:
        """将可选关节序列校验并转换为七元素浮点元组。"""
        if values is None:
            return None
        return finite_tuple(values, ARM_DOF, name)

    @staticmethod
    def _poses(
        left: Optional[CartesianPoseMM], right: Optional[CartesianPoseMM]
    ) -> Tuple[Optional[CartesianPoseMM], Optional[CartesianPoseMM]]:
        """校验至少一个左右臂位姿并归一化其中的四元数。"""
        if left is None and right is None:
            raise ValidationError("at least one of left or right must be supplied")
        if left is not None and not isinstance(left, CartesianPoseMM):
            raise ValidationError("left must be a CartesianPoseMM")
        if right is not None and not isinstance(right, CartesianPoseMM):
            raise ValidationError("right must be a CartesianPoseMM")
        return (
            left.validated() if left is not None else None,
            right.validated() if right is not None else None,
        )

    @staticmethod
    def _speed(value: float, maximum: float, name: str) -> float:
        """校验速度为有限非负数；零表示采用底层驱动默认值。"""
        result = finite_float(value, name)
        if not 0.0 <= result <= maximum:
            raise ValidationError(f"{name} must be in [0, {maximum}]")
        return result

    @staticmethod
    def _acceleration(value: float, name: str) -> float:
        """校验加速度为有限非负数；零表示采用底层驱动默认值。"""
        result = finite_float(value, name)
        if result < 0.0:
            raise ValidationError(f"{name} must be greater than or equal to zero")
        return result

    @staticmethod
    def _timeout(value: float) -> float:
        """校验指令超时时间为有限正数。"""
        result = finite_float(value, "timeout_sec")
        if result <= 0.0:
            raise ValidationError("timeout_sec must be greater than zero")
        return result

    def hold(self, *, timeout_sec: float = 5.0) -> CommandResult:
        """立即请求双臂保持当前位置并停止已有手臂运动。"""
        return self._backend.arm_hold(self._timeout(timeout_sec))

    def move_j(
        self,
        *,
        left: Optional[Sequence[float]] = None,
        right: Optional[Sequence[float]] = None,
        speed_rad_s: float = 0.0,
        acceleration_rad_s2: float = 0.0,
        relative: bool = False,
        wait: bool = True,
        timeout_sec: float = 30.0,
    ) -> CommandResult:
        """执行单臂或双臂关节空间运动。

        Args:
            left/right: 左右臂七个目标关节角，单位 rad；至少提供一侧。
            speed_rad_s: 关节速度；Shiloong 中 0 使用默认 0.628 rad/s。
            acceleration_rad_s2: 关节加速度；0 使用驱动默认值。
            relative: 为 True 时目标值表示相对当前关节的增量。
            wait: 是否等待底层运动结束。
            timeout_sec: 等待响应或运动完成的最长秒数。
        Returns:
            成功指令结果；底层失败时直接抛出统一异常。
        Raises:
            ValidationError: 关节数量、速度或超时时间不合法。
        """
        left_joints = self._joints(left, "left joints")
        right_joints = self._joints(right, "right joints")
        if left_joints is None and right_joints is None:
            raise ValidationError("at least one of left or right must be supplied")
        try:
            return self._backend.move_j(
                left_joints,
                right_joints,
                MotionMode.INCREMENTAL if relative else MotionMode.ABSOLUTE,
                self._speed(speed_rad_s, 3.14, "speed_rad_s"),
                self._acceleration(acceleration_rad_s2, "acceleration_rad_s2"),
                bool(wait),
                self._timeout(timeout_sec),
            )
        except CommandTimeoutError:
            self._best_effort_hold()
            raise

    def move_l(
        self,
        *,
        left: Optional[CartesianPoseMM] = None,
        right: Optional[CartesianPoseMM] = None,
        speed_mm_s: float = 0.0,
        acceleration_mm_s2: float = 0.0,
        relative: bool = False,
        left_psi_rad: float = 0.0,
        right_psi_rad: float = 0.0,
        wait: bool = True,
        timeout_sec: float = 30.0,
    ) -> CommandResult:
        """执行单臂或双臂法兰笛卡尔直线运动。

        Args:
            left/right: 左右臂法兰目标；位置单位 mm，至少提供一侧。
            speed_mm_s: 直线速度；Shiloong 中 0 使用默认 200 mm/s。
            acceleration_mm_s2: 直线加速度；0 使用驱动默认值。
            relative: 为 True 时位姿表示相对当前法兰的增量。
            left_psi_rad/right_psi_rad: 左右臂臂形角，单位 rad。
            wait: 是否等待运动结束；timeout_sec 为最长等待秒数。
        Returns:
            成功指令结果。
        Raises:
            ValidationError: 位姿、速度、臂形角或超时时间不合法。
        """
        left_pose, right_pose = self._poses(left, right)
        try:
            return self._backend.move_l(
                left_pose,
                right_pose,
                MotionMode.INCREMENTAL if relative else MotionMode.ABSOLUTE,
                self._speed(speed_mm_s, 1000.0, "speed_mm_s"),
                self._acceleration(acceleration_mm_s2, "acceleration_mm_s2"),
                finite_float(left_psi_rad, "left_psi_rad"),
                finite_float(right_psi_rad, "right_psi_rad"),
                bool(wait),
                self._timeout(timeout_sec),
            )
        except CommandTimeoutError:
            self._best_effort_hold()
            raise

    def move_p(
        self,
        *,
        left: Optional[CartesianPoseMM] = None,
        right: Optional[CartesianPoseMM] = None,
        speed_rad_s: float = 0.0,
        acceleration_rad_s2: float = 0.0,
        left_reference: Optional[Sequence[float]] = None,
        right_reference: Optional[Sequence[float]] = None,
        wait: bool = True,
        timeout_sec: float = 30.0,
    ) -> CommandResult:
        """对目标法兰位姿求逆解后执行关节运动。

        Args:
            left/right: 左右臂法兰目标；位置单位 mm，至少提供一侧。
            speed_rad_s: 逆解后速度；Shiloong 中 0 使用默认 0.628 rad/s。
            acceleration_rad_s2: 关节加速度；0 使用驱动默认值。
            left_reference/right_reference: 可选的七关节逆解参考值。
            wait: 是否等待运动结束；timeout_sec 为最长等待秒数。
        Returns:
            成功指令结果。
        Raises:
            ValidationError: 位姿、参考关节、速度或超时时间不合法。
        """
        left_pose, right_pose = self._poses(left, right)
        try:
            return self._backend.move_p(
                left_pose,
                right_pose,
                self._speed(speed_rad_s, 3.14, "speed_rad_s"),
                self._acceleration(acceleration_rad_s2, "acceleration_rad_s2"),
                self._joints(left_reference, "left_reference"),
                self._joints(right_reference, "right_reference"),
                bool(wait),
                self._timeout(timeout_sec),
            )
        except CommandTimeoutError:
            self._best_effort_hold()
            raise

    def move_p_offset(
        self,
        *,
        left: Optional[CartesianPoseMM] = None,
        right: Optional[CartesianPoseMM] = None,
        speed_rad_s: float = 0.0,
        acceleration_rad_s2: float = 0.0,
        wait: bool = True,
        timeout_sec: float = 30.0,
        hold_timeout_sec: float = 3.0,
        state_timeout_sec: float = 10.0,
        state_refresh_sec: float = 0.25,
    ) -> CommandResult:
        """从当前 TCP 位姿按位置增量和局部旋转偏移执行 MoveP。

        Args:
            left/right: arm_driver 三轴毫米增量和 TCP 局部旋转增量。
            speed_rad_s/acceleration_rad_s2: 逆解后的关节速度和加速度。
            wait: 是否等待运动完成；timeout_sec 为运动超时。
            hold_timeout_sec: 运动前 Hold 服务超时。
            state_timeout_sec: 等待当前 TCP 状态的超时。
            state_refresh_sec: Hold 后用于排除旧状态的等待秒数。
        Returns:
            MoveP 成功指令结果。
        Raises:
            RobotStateError: 所选手臂缺少当前 TCP 位姿。
            ValidationError: 偏移、速度或超时参数不合法。
        Notes:
            执行前会 Hold 双臂，因此要求左右臂均已使能。
        """
        left_offset, right_offset = self._poses(left, right)
        state, left_target, right_target = self._offset_targets(
            left_offset,
            right_offset,
            hold_timeout_sec=hold_timeout_sec,
            state_timeout_sec=state_timeout_sec,
            state_refresh_sec=state_refresh_sec,
        )
        return self.move_p(
            left=left_target,
            right=right_target,
            speed_rad_s=speed_rad_s,
            acceleration_rad_s2=acceleration_rad_s2,
            left_reference=(
                state.left_arm_joints if left_target is not None else None
            ),
            right_reference=(
                state.right_arm_joints if right_target is not None else None
            ),
            wait=wait,
            timeout_sec=timeout_sec,
        )

    def move_l_offset(
        self,
        *,
        left: Optional[CartesianPoseMM] = None,
        right: Optional[CartesianPoseMM] = None,
        speed_mm_s: float = 0.0,
        acceleration_mm_s2: float = 0.0,
        left_psi_rad: float = 0.0,
        right_psi_rad: float = 0.0,
        wait: bool = True,
        timeout_sec: float = 30.0,
        hold_timeout_sec: float = 3.0,
        state_timeout_sec: float = 10.0,
        state_refresh_sec: float = 0.25,
    ) -> CommandResult:
        """从当前 TCP 位姿按位置增量和局部旋转偏移执行 MoveL。

        Args:
            left/right: arm_driver 三轴毫米增量和 TCP 局部旋转增量。
            speed_mm_s/acceleration_mm_s2: TCP 直线速度和加速度。
            left_psi_rad/right_psi_rad: 左右臂臂形角。
            wait: 是否等待运动完成；timeout_sec 为运动超时。
            hold_timeout_sec: 运动前 Hold 服务超时。
            state_timeout_sec: 等待当前 TCP 状态的超时。
            state_refresh_sec: Hold 后用于排除旧状态的等待秒数。
        Returns:
            MoveL 成功指令结果。
        Raises:
            RobotStateError: 所选手臂缺少当前 TCP 位姿。
            ValidationError: 偏移、速度或超时参数不合法。
        Notes:
            执行前会 Hold 双臂，因此要求左右臂均已使能。
        """
        left_offset, right_offset = self._poses(left, right)
        _, left_target, right_target = self._offset_targets(
            left_offset,
            right_offset,
            hold_timeout_sec=hold_timeout_sec,
            state_timeout_sec=state_timeout_sec,
            state_refresh_sec=state_refresh_sec,
        )
        return self.move_l(
            left=left_target,
            right=right_target,
            speed_mm_s=speed_mm_s,
            acceleration_mm_s2=acceleration_mm_s2,
            relative=False,
            left_psi_rad=left_psi_rad,
            right_psi_rad=right_psi_rad,
            wait=wait,
            timeout_sec=timeout_sec,
        )

    def _offset_targets(
        self,
        left_offset: Optional[CartesianPoseMM],
        right_offset: Optional[CartesianPoseMM],
        *,
        hold_timeout_sec: float,
        state_timeout_sec: float,
        state_refresh_sec: float,
    ) -> Tuple[
        SystemState,
        Optional[CartesianPoseMM],
        Optional[CartesianPoseMM],
    ]:
        """Hold 后读取当前位姿并生成所选双臂局部偏移的绝对目标。"""
        hold_timeout = self._timeout(hold_timeout_sec)
        state_timeout = self._timeout(state_timeout_sec)
        refresh = finite_float(state_refresh_sec, "state_refresh_sec")
        if refresh < 0.0:
            raise ValidationError("state_refresh_sec must be >= 0")
        self.hold(timeout_sec=hold_timeout)
        if refresh > 0.0:
            time.sleep(refresh)
        state = self._backend.get_system_state(
            max_age_sec=max(refresh, 0.001),
            wait_timeout_sec=state_timeout,
        )
        if left_offset is not None and state.left_arm_pose is None:
            raise RobotStateError("current left-arm TCP pose is unavailable")
        if right_offset is not None and state.right_arm_pose is None:
            raise RobotStateError("current right-arm TCP pose is unavailable")
        left_target = (
            state.left_arm_pose.offset_local(left_offset)
            if left_offset is not None
            else None
        )
        right_target = (
            state.right_arm_pose.offset_local(right_offset)
            if right_offset is not None
            else None
        )
        return state, left_target, right_target

    def _best_effort_hold(self) -> None:
        """在手臂指令超时后尽力发送 Hold，且不覆盖原始异常。"""
        try:
            self._backend.arm_hold(2.0)
        except Exception:
            pass
