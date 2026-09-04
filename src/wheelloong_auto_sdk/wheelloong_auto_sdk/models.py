"""ROS-independent public value objects used by the SDK."""

from dataclasses import dataclass, replace
from enum import Enum, IntEnum
import math
import time
from typing import Callable, Optional, Sequence, Tuple

from .errors import ValidationError


ARM_DOF = 7


def finite_float(value: object, name: str) -> float:
    """将输入转换为有限浮点数，否则抛出参数校验异常。"""
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise ValidationError(f"{name} must be a number") from exc
    if not math.isfinite(result):
        raise ValidationError(f"{name} must be finite")
    return result


def finite_tuple(
    values: Sequence[object], length: int, name: str
) -> Tuple[float, ...]:
    """将定长序列转换为完全由有限浮点数组成的元组。"""
    try:
        items = tuple(values)
    except TypeError as exc:
        raise ValidationError(f"{name} must be a sequence") from exc
    if len(items) != length:
        raise ValidationError(f"{name} must contain exactly {length} values")
    return tuple(finite_float(value, f"{name}[{index}]") for index, value in enumerate(items))


class ArmSide(IntEnum):
    """定义左臂、右臂和双臂选择值。"""
    DUAL = -1
    LEFT = 0
    RIGHT = 1


class MotionMode(IntEnum):
    """定义机械臂绝对运动和增量运动模式。"""
    ABSOLUTE = 0
    INCREMENTAL = 1


class ControlMode(IntEnum):
    """定义 SDK 使用的 AUTO 与 IDLE 系统控制模式。"""
    AUTO = 2
    IDLE = 99


class NavigationMode(Enum):
    """定义默认导航及三种精确导航策略。"""
    DEFAULT = "default"
    PRECISE_BACKWARD = "precise_backward"
    PRECISE_FORWARD = "precise_forward"
    PRECISE = "precise"


@dataclass(frozen=True)
class AxisSelection:
    """描述一个 AUTO 会话需要使能的手臂、头部和腰部轴。"""
    left_arm: bool = False
    right_arm: bool = False
    head_pitch: bool = False
    head_yaw: bool = False
    waist_pitch: bool = False
    waist_lift: bool = False

    @classmethod
    def all(cls) -> "AxisSelection":
        """创建选择全部六组运动轴的配置。"""
        return cls(True, True, True, True, True, True)

    @classmethod
    def none(cls) -> "AxisSelection":
        """创建不选择任何运动轴的配置。"""
        return cls()

    @property
    def is_empty(self) -> bool:
        """判断当前配置是否未选择任何运动轴。"""
        return not any(
            (
                self.left_arm,
                self.right_arm,
                self.head_pitch,
                self.head_yaw,
                self.waist_pitch,
                self.waist_lift,
            )
        )

    def satisfied_by(self, state: "SystemState") -> bool:
        """判断系统状态是否已使能当前配置要求的全部轴。"""
        checks = (
            (self.left_arm, state.left_arm_enabled),
            (self.right_arm, state.right_arm_enabled),
            (self.head_pitch, state.head_pitch_enabled),
            (self.head_yaw, state.head_yaw_enabled),
            (self.waist_pitch, state.waist_pitch_enabled),
            (self.waist_lift, state.waist_lift_enabled),
        )
        return all((not required) or actual for required, actual in checks)


@dataclass(frozen=True)
class CartesianPoseMM:
    """表示相对胸部坐标系的机械臂法兰位姿，位置单位为毫米。"""

    x_mm: float
    y_mm: float
    z_mm: float
    qx: float = 0.0
    qy: float = 0.0
    qz: float = 0.0
    qw: float = 1.0
    frame_id: str = "arm_driver"

    def validated(self) -> "CartesianPoseMM":
        """校验数值并返回四元数归一化后的新位姿。"""
        values = finite_tuple(
            (self.x_mm, self.y_mm, self.z_mm, self.qx, self.qy, self.qz, self.qw),
            7,
            "arm pose",
        )
        norm = math.sqrt(sum(value * value for value in values[3:]))
        if norm < 1e-9:
            raise ValidationError("arm pose quaternion must not be zero")
        return replace(
            self,
            x_mm=values[0],
            y_mm=values[1],
            z_mm=values[2],
            qx=values[3] / norm,
            qy=values[4] / norm,
            qz=values[5] / norm,
            qw=values[6] / norm,
            frame_id=str(self.frame_id or "arm_driver"),
        )

    @classmethod
    def from_rpy(
        cls,
        x_mm: float,
        y_mm: float,
        z_mm: float,
        roll_rad: float,
        pitch_rad: float,
        yaw_rad: float,
        *,
        frame_id: str = "arm_driver",
    ) -> "CartesianPoseMM":
        """由毫米位置和弧度制 RPY 欧拉角创建法兰位姿。"""
        roll, pitch, yaw = finite_tuple(
            (roll_rad, pitch_rad, yaw_rad), 3, "roll/pitch/yaw"
        )
        cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
        cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
        cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
        return cls(
            x_mm,
            y_mm,
            z_mm,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy,
            frame_id,
        ).validated()


@dataclass(frozen=True)
class NavigationPose:
    """表示平面导航目标，位置单位为米，偏航角单位为弧度。"""

    x_m: float
    y_m: float
    yaw_rad: float
    z_m: float = 0.0
    frame_id: str = "map"

    def validated(self) -> "NavigationPose":
        """校验导航位姿数值并补全默认坐标系。"""
        values = finite_tuple(
            (self.x_m, self.y_m, self.yaw_rad, self.z_m), 4, "navigation pose"
        )
        return replace(
            self,
            x_m=values[0],
            y_m=values[1],
            yaw_rad=values[2],
            z_m=values[3],
            frame_id=str(self.frame_id or "map"),
        )


@dataclass(frozen=True)
class CommandResult:
    """表示服务指令成功返回后的统一结果。"""
    operation: str
    error_code: int = 0
    message: str = ""

    @property
    def ok(self) -> bool:
        """判断底层错误码是否为零。"""
        return self.error_code == 0


@dataclass(frozen=True)
class SystemState:
    """保存从 SystemInfo 转换得到的 ROS 无关机器人状态快照。"""
    received_monotonic: float
    heartbeat: int
    control_mode: int
    errors: Tuple[str, ...] = ()
    warnings: Tuple[str, ...] = ()
    left_arm_enabled: bool = False
    right_arm_enabled: bool = False
    head_pitch_enabled: bool = False
    head_yaw_enabled: bool = False
    waist_pitch_enabled: bool = False
    waist_lift_enabled: bool = False
    left_arm_joints: Tuple[float, ...] = (0.0,) * ARM_DOF
    right_arm_joints: Tuple[float, ...] = (0.0,) * ARM_DOF
    left_arm_pose: Optional[CartesianPoseMM] = None
    right_arm_pose: Optional[CartesianPoseMM] = None
    head_pitch_rad: float = 0.0
    head_yaw_rad: float = 0.0
    waist_pitch_rad: float = 0.0
    waist_lift_mm: float = 0.0
    left_gripper: float = 0.0
    right_gripper: float = 0.0

    @property
    def age_sec(self) -> float:
        """返回该状态快照距当前单调时钟的秒数。"""
        return max(0.0, time.monotonic() - self.received_monotonic)


@dataclass(frozen=True)
class NavigationFeedback:
    """保存导航过程中的距离、耗时、恢复次数和剩余点数。"""
    distance_remaining_m: float = 0.0
    estimated_time_remaining_sec: float = 0.0
    navigation_time_sec: float = 0.0
    recoveries: int = 0
    poses_remaining: Optional[int] = None


@dataclass(frozen=True)
class NavigationResult:
    """表示导航目标的最终状态及是否成功。"""
    status: str
    succeeded: bool


StatePredicate = Callable[[SystemState], bool]
