"""Top-level SDK facade and runtime factories."""

from typing import Optional, Sequence

from .arms import Arms
from .backend.base import RobotBackend
from .body import Body
from .errors import ValidationError
from .gripper import Gripper
from .models import AxisSelection, SystemState, finite_float
from .navigation import Navigation
from .session import AutoSession
from .system import System
from .voice import Voice


class Robot:
    """作为任务程序唯一入口，聚合各能力并隐藏具体通信后端。"""

    def __init__(self, backend: RobotBackend) -> None:
        """使用一个符合 RobotBackend 契约的实例组装所有子接口。"""
        if not isinstance(backend, RobotBackend):
            raise TypeError("backend must implement RobotBackend")
        self._backend = backend
        self.arms = Arms(backend)
        self.body = Body(backend)
        self.gripper = Gripper(backend)
        self.navigation = Navigation(backend)
        self.system = System(backend)
        self.voice = Voice(backend)
        self._closed = False

    @classmethod
    def from_node(cls, node) -> "Robot":
        """借用任务工程已有 Node 创建直连 ROS 机器人对象。

        Args:
            node: 由调用方拥有且会持续 spin 的 rclpy Node。
        Returns:
            不拥有 Node、Context 或 Executor 的机器人对象。
        Notes:
            同步接口若在回调内调用，应使用至少两线程 Executor。
        """
        from .backend.ros2 import ROS2Backend

        return cls(ROS2Backend(node))

    @classmethod
    def standalone(
        cls,
        *,
        node_name: str = "wheelloong_auto_sdk",
        ros_args: Optional[Sequence[str]] = None,
    ) -> "Robot":
        """创建拥有私有 ROS Context、Node 和执行器的机器人对象。

        Args:
            node_name: SDK 私有 ROS Node 的名称。
            ros_args: 可选 ROS 初始化参数。
        Returns:
            可供普通 Python 脚本直接使用的机器人对象。
        """
        from .backend.ros2 import ROS2Backend

        return cls(ROS2Backend.standalone(node_name=node_name, ros_args=ros_args))

    @classmethod
    def with_backend(cls, backend: RobotBackend) -> "Robot":
        """注入 Mock、直连 ROS 或未来 Server Backend 创建机器人对象。"""
        return cls(backend)

    @property
    def node(self):
        """返回后端使用的 ROS Node；非 ROS 后端可返回 None。"""
        return self._backend.node

    def state(
        self, *, max_age_sec: float = 0.5, wait_timeout_sec: float = 2.0
    ) -> SystemState:
        """获取满足新鲜度要求的机器人系统状态。

        Args:
            max_age_sec: 可接受状态距当前时间的最大秒数。
            wait_timeout_sec: 等待新状态到达的最长秒数。
        Returns:
            不包含 ROS 消息类型的 SystemState 快照。
        Raises:
            RobotStateError: 截止时间前没有新鲜状态。
            ValidationError: 时间参数不是正数。
        """
        return self.system.state(
            max_age_sec=max_age_sec, wait_timeout_sec=wait_timeout_sec
        )

    def auto_session(
        self,
        *,
        required_axes: Optional[AxisSelection] = None,
        state_timeout_sec: float = 5.0,
        service_timeout_sec: float = 5.0,
        max_state_age_sec: float = 0.5,
    ) -> AutoSession:
        """创建负责 AUTO、使能、Hold 和退出清理的上下文管理器。

        Args:
            required_axes: 会话独占并使能的轴；None 表示全部轴。
            state_timeout_sec: 等待控制模式或使能状态确认的秒数。
            service_timeout_sec: 等待底层服务响应的秒数。
            max_state_age_sec: 进入会话时允许的状态最大年龄。
        Returns:
            尚未进入的 AutoSession；应配合 with 使用。
        Raises:
            ValidationError: 轴配置或时间参数不合法。
        """
        axes = AxisSelection.all() if required_axes is None else required_axes
        if not isinstance(axes, AxisSelection):
            raise ValidationError("required_axes must be an AxisSelection")
        state_timeout = finite_float(state_timeout_sec, "state_timeout_sec")
        service_timeout = finite_float(service_timeout_sec, "service_timeout_sec")
        max_age = finite_float(max_state_age_sec, "max_state_age_sec")
        if min(state_timeout, service_timeout, max_age) <= 0.0:
            raise ValidationError("AUTO session timeouts must be greater than zero")
        return AutoSession(self, axes, state_timeout, service_timeout, max_age)

    def close(self) -> None:
        """取消活动导航并按后端所有权规则释放资源；可重复调用。"""
        if self._closed:
            return
        self._closed = True
        self.navigation.cancel_all(timeout_sec=5.0)
        self._backend.close()

    def __enter__(self) -> "Robot":
        """进入 Robot 资源上下文并返回自身。"""
        return self

    def __exit__(self, exc_type, exc, traceback) -> bool:
        """退出资源上下文时关闭 Robot，且不吞掉业务异常。"""
        self.close()
        return False
