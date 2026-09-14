"""Robot-model limits and reusable named joint poses."""

from dataclasses import dataclass
import math
from typing import Sequence, Tuple

from .errors import ValidationError
from .models import ARM_DOF, finite_tuple


JointDegrees = Tuple[float, ...]
JointRadians = Tuple[float, ...]
JointLimitsDegrees = Tuple[Tuple[float, float], ...]


@dataclass(frozen=True)
class RobotProfile:
    """保存一种机器人机型的标准姿态和机械限位。"""

    name: str
    work_left_deg: JointDegrees
    work_right_deg: JointDegrees
    return_left_deg: JointDegrees
    return_right_deg: JointDegrees
    arm_limits_deg: JointLimitsDegrees
    head_pitch_limits_deg: Tuple[float, float]
    head_yaw_limits_deg: Tuple[float, float]
    waist_pitch_limits_deg: Tuple[float, float]
    waist_lift_limits_mm: Tuple[float, float]

    def work_arm_joints(self) -> Tuple[JointRadians, JointRadians]:
        """返回左右臂标准工作位，单位 rad。

        Returns:
            左臂和右臂七关节弧度元组。
        """
        return (
            self._radians(self.work_left_deg),
            self._radians(self.work_right_deg),
        )

    def return_arm_joints(self) -> Tuple[JointRadians, JointRadians]:
        """返回左右臂标准结束位，单位 rad。

        Returns:
            左臂和右臂七关节弧度元组。
        """
        return (
            self._radians(self.return_left_deg),
            self._radians(self.return_right_deg),
        )

    def validate_arm_degrees(
        self, values: Sequence[float], *, side: str
    ) -> JointDegrees:
        """校验一侧七关节角处于该机型机械限位内。

        Args:
            values: 七个度制关节角。
            side: 用于错误信息的 left 或 right 标识。
        Returns:
            归一化后的七元素浮点元组。
        Raises:
            ValidationError: 数量、数值或机械范围不合法。
        """
        joints = finite_tuple(values, ARM_DOF, f"{side} arm joints")
        for index, (target, limits) in enumerate(
            zip(joints, self.arm_limits_deg), start=1
        ):
            if not limits[0] <= target <= limits[1]:
                raise ValidationError(
                    f"{side} arm J{index} must be in "
                    f"[{limits[0]}, {limits[1]}] degrees"
                )
        return joints

    @staticmethod
    def _radians(values: Sequence[float]) -> JointRadians:
        return tuple(math.radians(value) for value in values)


SHILOONG_PROFILE = RobotProfile(
    name="shiloong",
    work_left_deg=(10.0, 77.0, -80.0, 20.0, -25.0, 10.0, 10.0),
    work_right_deg=(-10.0, 77.0, 80.0, 20.0, 25.0, -10.0, 10.0),
    return_left_deg=(0.0, 85.0, -2.0, 3.0, -25.0, 0.0, 0.0),
    return_right_deg=(0.0, 85.0, -2.0, 3.0, 25.0, 0.0, 0.0),
    arm_limits_deg=(
        (-155.0, 155.0),
        (-100.0, 100.0),
        (-158.0, 158.0),
        (-58.0, 123.0),
        (-158.0, 158.0),
        (-42.0, 55.0),
        (-90.0, 90.0),
    ),
    head_pitch_limits_deg=(-7.0, 60.0),
    head_yaw_limits_deg=(-90.0, 90.0),
    waist_pitch_limits_deg=(-60.0, 40.0),
    waist_lift_limits_mm=(-380.0, 0.0),
)

WHEELLOONG_PROFILE = RobotProfile(
    name="wheelloong",
    work_left_deg=(0.0,) * ARM_DOF,
    work_right_deg=(0.0,) * ARM_DOF,
    return_left_deg=(0.0,) * ARM_DOF,
    return_right_deg=(0.0,) * ARM_DOF,
    arm_limits_deg=SHILOONG_PROFILE.arm_limits_deg,
    head_pitch_limits_deg=SHILOONG_PROFILE.head_pitch_limits_deg,
    head_yaw_limits_deg=SHILOONG_PROFILE.head_yaw_limits_deg,
    waist_pitch_limits_deg=SHILOONG_PROFILE.waist_pitch_limits_deg,
    waist_lift_limits_mm=SHILOONG_PROFILE.waist_lift_limits_mm,
)

ROBOT_PROFILES = {
    SHILOONG_PROFILE.name: SHILOONG_PROFILE,
    WHEELLOONG_PROFILE.name: WHEELLOONG_PROFILE,
}


def get_robot_profile(name: str) -> RobotProfile:
    """按机型名返回标准配置。

    Args:
        name: wheelloong 或 shiloong，不区分大小写。
    Returns:
        对应的不可变 RobotProfile。
    Raises:
        ValidationError: 机型名不受支持。
    """
    try:
        return ROBOT_PROFILES[str(name).strip().lower()]
    except KeyError as exc:
        supported = ", ".join(sorted(ROBOT_PROFILES))
        raise ValidationError(
            f"unsupported robot profile: {name}; expected one of {supported}"
        ) from exc
