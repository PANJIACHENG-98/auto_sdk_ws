"""In-memory backend for task development and SDK tests without a robot."""

from dataclasses import replace
import time
from typing import List, Optional, Sequence, Tuple

from .base import BackendNavigationHandle, JointTuple, RobotBackend
from ..errors import CommandTimeoutError
from ..models import (
    AxisSelection,
    CartesianPoseMM,
    CommandResult,
    MotionMode,
    NavigationFeedback,
    NavigationMode,
    NavigationPose,
    NavigationResult,
    StatePredicate,
    SystemState,
)


class _MockNavigationHandle(BackendNavigationHandle):
    """模拟可等待、可取消的活动导航句柄。"""

    def __init__(self) -> None:
        """初始化为尚未结束，首次 wait 时模拟成功。"""
        self._result: Optional[NavigationResult] = None

    @property
    def feedback(self) -> Optional[NavigationFeedback]:
        """返回一个零值模拟导航反馈。"""
        return NavigationFeedback()

    @property
    def done(self) -> bool:
        """报告模拟目标是否已等待完成或被取消。"""
        return self._result is not None

    def wait(self, timeout_sec: Optional[float]) -> NavigationResult:
        """首次等待时把模拟导航切换为成功。"""
        if self._result is None:
            self._result = NavigationResult("SUCCEEDED", True)
        return self._result

    def cancel(self, timeout_sec: float) -> NavigationResult:
        """把模拟导航切换为 CANCELED 并返回结果。"""
        if self._result is None:
            self._result = NavigationResult("CANCELED", False)
        return self._result


class MockBackend(RobotBackend):
    """记录语义调用并模拟立即成功，用于无机器人任务测试。"""

    def __init__(self) -> None:
        """创建空调用记录和初始 IDLE 系统状态。"""
        self.calls: List[Tuple[str, dict]] = []
        self.closed = False
        self.state = SystemState(
            received_monotonic=time.monotonic(),
            heartbeat=1,
            control_mode=99,
            left_arm_pose=CartesianPoseMM(0.0, 0.0, 0.0),
            right_arm_pose=CartesianPoseMM(0.0, 0.0, 0.0),
        )
        self.navigation_pose = NavigationPose(0.0, 0.0, 0.0)

    def _record(self, operation: str, **values) -> CommandResult:
        """记录一次操作参数并返回成功结果。"""
        self.calls.append((operation, values))
        return CommandResult(operation)

    def close(self) -> None:
        """将模拟后端标记为已关闭。"""
        self.closed = True

    def get_system_state(self, max_age_sec: float, wait_timeout_sec: float) -> SystemState:
        """刷新接收时间并立即返回当前模拟状态。"""
        self.state = replace(self.state, received_monotonic=time.monotonic())
        return self.state

    def wait_system_state(
        self,
        predicate: StatePredicate,
        timeout_sec: float,
        description: str,
    ) -> SystemState:
        """立即检查状态谓词，不满足时抛出模拟超时。"""
        self.state = replace(self.state, received_monotonic=time.monotonic())
        if not predicate(self.state):
            raise CommandTimeoutError(f"mock state did not satisfy {description}")
        return self.state

    def set_control_mode(self, mode: int, timeout_sec: float) -> CommandResult:
        """更新模拟控制模式并记录操作。"""
        self.state = replace(
            self.state,
            received_monotonic=time.monotonic(),
            control_mode=int(mode),
        )
        return self._record("set_control_mode", mode=int(mode))

    def set_enabled(self, axes: AxisSelection, timeout_sec: float) -> CommandResult:
        """更新模拟轴使能状态并记录操作。"""
        self.state = replace(
            self.state,
            received_monotonic=time.monotonic(),
            left_arm_enabled=axes.left_arm,
            right_arm_enabled=axes.right_arm,
            head_pitch_enabled=axes.head_pitch,
            head_yaw_enabled=axes.head_yaw,
            waist_pitch_enabled=axes.waist_pitch,
            waist_lift_enabled=axes.waist_lift,
        )
        return self._record("set_enabled", axes=axes)

    def arm_hold(self, timeout_sec: float) -> CommandResult:
        """记录一次模拟双臂 Hold 操作。"""
        return self._record("arm_hold")

    def move_j(
        self,
        left: Optional[JointTuple],
        right: Optional[JointTuple],
        mode: MotionMode,
        speed_rad_s: float,
        acceleration_rad_s2: float,
        wait: bool,
        timeout_sec: float,
    ) -> CommandResult:
        """记录模拟 MoveJ 参数并返回成功。"""
        return self._record(
            "move_j",
            left=left,
            right=right,
            mode=mode,
            speed=speed_rad_s,
            acceleration=acceleration_rad_s2,
            wait=wait,
        )

    def move_l(
        self,
        left: Optional[CartesianPoseMM],
        right: Optional[CartesianPoseMM],
        mode: MotionMode,
        speed_mm_s: float,
        acceleration_mm_s2: float,
        left_psi_rad: float,
        right_psi_rad: float,
        wait: bool,
        timeout_sec: float,
    ) -> CommandResult:
        """记录模拟 MoveL 参数并返回成功。"""
        if mode == MotionMode.ABSOLUTE:
            self.state = replace(
                self.state,
                left_arm_pose=left or self.state.left_arm_pose,
                right_arm_pose=right or self.state.right_arm_pose,
            )
        return self._record(
            "move_l",
            left=left,
            right=right,
            mode=mode,
            speed=speed_mm_s,
            acceleration=acceleration_mm_s2,
            left_psi=left_psi_rad,
            right_psi=right_psi_rad,
            wait=wait,
        )

    def move_p(
        self,
        left: Optional[CartesianPoseMM],
        right: Optional[CartesianPoseMM],
        speed_rad_s: float,
        acceleration_rad_s2: float,
        left_reference: Optional[JointTuple],
        right_reference: Optional[JointTuple],
        wait: bool,
        timeout_sec: float,
    ) -> CommandResult:
        """记录模拟 MoveP 参数并返回成功。"""
        self.state = replace(
            self.state,
            left_arm_pose=left or self.state.left_arm_pose,
            right_arm_pose=right or self.state.right_arm_pose,
        )
        return self._record(
            "move_p",
            left=left,
            right=right,
            speed=speed_rad_s,
            acceleration=acceleration_rad_s2,
            left_reference=left_reference,
            right_reference=right_reference,
            wait=wait,
        )

    def move_body(
        self,
        head_pitch_rad: Optional[float],
        head_yaw_rad: Optional[float],
        waist_pitch_rad: Optional[float],
        waist_lift_mm: Optional[float],
        speed_percent: float,
        wait: bool,
        timeout_sec: float,
    ) -> CommandResult:
        """记录模拟头腰运动参数并返回成功。"""
        return self._record(
            "move_body",
            head_pitch=head_pitch_rad,
            head_yaw=head_yaw_rad,
            waist_pitch=waist_pitch_rad,
            waist_lift=waist_lift_mm,
            speed=speed_percent,
            wait=wait,
        )

    def set_gripper(
        self,
        side_index: int,
        position: float,
        speed: float,
        torque: float,
        wait: bool,
        tolerance: float,
        timeout_sec: float,
    ) -> CommandResult:
        """更新模拟夹爪状态、记录参数并返回成功。"""
        left = position if side_index in (-1, 0) else self.state.left_gripper
        right = position if side_index in (-1, 1) else self.state.right_gripper
        self.state = replace(
            self.state,
            received_monotonic=time.monotonic(),
            left_gripper=left,
            right_gripper=right,
        )
        return self._record(
            "set_gripper", side=side_index, position=position, speed=speed, torque=torque
        )

    def speak(
        self, text: str, wait: bool, speech_timeout_sec: float, call_timeout_sec: float
    ) -> CommandResult:
        """记录模拟文本播报参数并返回成功。"""
        return self._record("speak", text=text, wait=wait)

    def play_audio(
        self, file_name: str, play_count: int, wait: bool, timeout_sec: float
    ) -> CommandResult:
        """记录模拟音频播放参数并返回成功。"""
        return self._record(
            "play_audio", file_name=file_name, play_count=play_count, wait=wait
        )

    def navigate_to(
        self,
        pose: NavigationPose,
        mode: NavigationMode,
        behavior_tree: str,
        server_timeout_sec: float,
    ) -> BackendNavigationHandle:
        """记录单点导航并返回立即成功的模拟句柄。"""
        self._record("navigate_to", pose=pose, mode=mode, behavior_tree=behavior_tree)
        return _MockNavigationHandle()

    def navigate_through(
        self,
        poses: Sequence[NavigationPose],
        mode: NavigationMode,
        behavior_tree: str,
        server_timeout_sec: float,
    ) -> BackendNavigationHandle:
        """记录多点导航并返回立即成功的模拟句柄。"""
        self._record(
            "navigate_through", poses=tuple(poses), mode=mode, behavior_tree=behavior_tree
        )
        return _MockNavigationHandle()

    def clear_local_costmap(self, timeout_sec: float) -> CommandResult:
        """记录一次模拟局部代价地图清理。"""
        return self._record("clear_local_costmap", timeout_sec=timeout_sec)

    def clear_global_costmap(self, timeout_sec: float) -> CommandResult:
        """记录一次模拟全局代价地图清理。"""
        return self._record("clear_global_costmap", timeout_sec=timeout_sec)

    def get_navigation_pose(
        self,
        global_frame: str,
        base_frame: str,
        timeout_sec: float,
    ) -> NavigationPose:
        """记录查询参数并返回可配置的模拟导航位姿。"""
        self._record(
            "get_navigation_pose",
            global_frame=global_frame,
            base_frame=base_frame,
            timeout_sec=timeout_sec,
        )
        return replace(self.navigation_pose, frame_id=global_frame)
