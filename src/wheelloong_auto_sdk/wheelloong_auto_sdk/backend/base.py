"""ROS-independent backend contract behind the public SDK."""

from abc import ABC, abstractmethod
from typing import Optional, Sequence, Tuple

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


JointTuple = Tuple[float, ...]


class BackendNavigationHandle(ABC):
    """定义各通信后端必须实现的导航目标句柄契约。"""

    @property
    @abstractmethod
    def feedback(self) -> Optional[NavigationFeedback]:
        """返回最近导航反馈，尚未收到时返回 None。"""
        raise NotImplementedError

    @property
    @abstractmethod
    def done(self) -> bool:
        """判断导航目标是否已经到达最终状态。"""
        raise NotImplementedError

    @abstractmethod
    def wait(self, timeout_sec: Optional[float]) -> NavigationResult:
        """等待导航目标结束并返回后端无关结果。"""
        raise NotImplementedError

    @abstractmethod
    def cancel(self, timeout_sec: float) -> NavigationResult:
        """取消目标、等待终态并返回后端无关结果。"""
        raise NotImplementedError


class RobotBackend(ABC):
    """定义不包含 ROS 消息类型的语义化机器人后端契约。"""

    @property
    def node(self):
        """返回后端 Node；不使用 ROS 的后端返回 None。"""
        return None

    @abstractmethod
    def close(self) -> None:
        """按所有权规则释放后端资源。"""
        raise NotImplementedError

    @abstractmethod
    def get_system_state(self, max_age_sec: float, wait_timeout_sec: float) -> SystemState:
        """获取满足最大年龄要求的系统状态快照。"""
        raise NotImplementedError

    @abstractmethod
    def wait_system_state(
        self,
        predicate: StatePredicate,
        timeout_sec: float,
        description: str,
    ) -> SystemState:
        """等待系统状态满足指定谓词或达到截止时间。"""
        raise NotImplementedError

    @abstractmethod
    def set_control_mode(self, mode: int, timeout_sec: float) -> CommandResult:
        """设置系统控制模式。"""
        raise NotImplementedError

    @abstractmethod
    def set_enabled(self, axes: AxisSelection, timeout_sec: float) -> CommandResult:
        """设置六组手臂、头部和腰部轴的使能状态。"""
        raise NotImplementedError

    @abstractmethod
    def arm_hold(self, timeout_sec: float) -> CommandResult:
        """请求双臂停止现有运动并保持当前位置。"""
        raise NotImplementedError

    @abstractmethod
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
        """执行语义化单臂或双臂关节运动。"""
        raise NotImplementedError

    @abstractmethod
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
        """执行语义化单臂或双臂笛卡尔直线运动。"""
        raise NotImplementedError

    @abstractmethod
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
        """执行语义化机械臂逆解点到点运动。"""
        raise NotImplementedError

    @abstractmethod
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
        """控制明确选择的头部和腰部关节。"""
        raise NotImplementedError

    @abstractmethod
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
        """设置夹爪目标并可等待实际位置确认。"""
        raise NotImplementedError

    @abstractmethod
    def speak(
        self, text: str, wait: bool, speech_timeout_sec: float, call_timeout_sec: float
    ) -> CommandResult:
        """请求文本转语音播报。"""
        raise NotImplementedError

    @abstractmethod
    def play_audio(
        self, file_name: str, play_count: int, wait: bool, timeout_sec: float
    ) -> CommandResult:
        """请求播放指定本地音频文件。"""
        raise NotImplementedError

    @abstractmethod
    def navigate_to(
        self,
        pose: NavigationPose,
        mode: NavigationMode,
        behavior_tree: str,
        server_timeout_sec: float,
    ) -> BackendNavigationHandle:
        """下发单点导航并返回后端导航句柄。"""
        raise NotImplementedError

    @abstractmethod
    def navigate_through(
        self,
        poses: Sequence[NavigationPose],
        mode: NavigationMode,
        behavior_tree: str,
        server_timeout_sec: float,
    ) -> BackendNavigationHandle:
        """下发多点导航并返回后端导航句柄。"""
        raise NotImplementedError

    @abstractmethod
    def clear_local_costmap(self, timeout_sec: float) -> CommandResult:
        """清除 Nav2 局部代价地图。"""
        raise NotImplementedError

    @abstractmethod
    def clear_global_costmap(self, timeout_sec: float) -> CommandResult:
        """清除 Nav2 全局代价地图。"""
        raise NotImplementedError

    @abstractmethod
    def get_navigation_pose(
        self,
        global_frame: str,
        base_frame: str,
        timeout_sec: float,
    ) -> NavigationPose:
        """读取全局坐标系中的机器人当前平面位姿。"""
        raise NotImplementedError
