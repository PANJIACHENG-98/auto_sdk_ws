"""Public navigation API with controllable handles and map helpers."""

from pathlib import Path
from typing import Callable, List, Optional, Sequence, Union

from .backend.base import BackendNavigationHandle, RobotBackend
from .errors import (
    CommandTimeoutError,
    NavigationCancelledError,
    NavigationError,
    ValidationError,
)
from .models import (
    CommandResult,
    NavigationFeedback,
    NavigationMode,
    NavigationPose,
    NavigationResult,
    finite_float,
)


RestartNavigation = Callable[[float], BackendNavigationHandle]


class NavigationHandle:
    """封装已接受的导航目标，支持暂停、恢复、等待和取消。"""

    def __init__(
        self,
        implementation: BackendNavigationHandle,
        restart: RestartNavigation,
    ) -> None:
        """保存后端句柄和用于恢复原目标的重新下发函数。"""
        self._implementation = implementation
        self._restart = restart
        self._paused = False
        self._pause_result: Optional[NavigationResult] = None
        self._last_feedback: Optional[NavigationFeedback] = None

    @property
    def feedback(self) -> Optional[NavigationFeedback]:
        """返回当前目标或暂停前保存的最近一次反馈。"""
        current = self._implementation.feedback
        if current is not None:
            self._last_feedback = current
        return self._last_feedback

    @property
    def done(self) -> bool:
        """判断逻辑导航任务是否已结束；暂停状态不算结束。"""
        return False if self._paused else self._implementation.done

    @property
    def paused(self) -> bool:
        """判断当前导航目标是否已由 SDK 暂停。"""
        return self._paused

    def wait(
        self,
        timeout_sec: Optional[float] = None,
        *,
        cancel_on_timeout: bool = True,
        cancel_timeout_sec: float = 5.0,
    ) -> NavigationResult:
        """等待成功终态，可在等待超时后取消目标。

        Args:
            timeout_sec: 最长等待秒数；None 表示不设截止时间。
            cancel_on_timeout: 超时后是否主动取消目标。
            cancel_timeout_sec: 等待取消确认和最终状态的秒数。
        Returns:
            状态为 SUCCEEDED 的导航结果。
        Raises:
            CommandTimeoutError: 等待超过截止时间。
            NavigationError: 目标暂停、取消、拒绝或中止。
        """
        if self._paused:
            raise NavigationError(
                "navigation is paused; resume or cancel it before wait"
            )
        timeout = None
        if timeout_sec is not None:
            timeout = finite_float(timeout_sec, "timeout_sec")
            if timeout <= 0.0:
                raise ValidationError("timeout_sec must be greater than zero")
        try:
            result = self._implementation.wait(timeout)
        except CommandTimeoutError as original:
            if not cancel_on_timeout:
                raise
            cancel_timeout = finite_float(
                cancel_timeout_sec, "cancel_timeout_sec"
            )
            if cancel_timeout <= 0.0:
                raise ValidationError(
                    "cancel_timeout_sec must be greater than zero"
                )
            terminal = self._implementation.cancel(cancel_timeout)
            raise CommandTimeoutError(
                "navigation timed out; cancellation confirmed with "
                f"status {terminal.status}"
            ) from original
        return self._require_success(result)

    def pause(self, *, timeout_sec: float = 5.0) -> NavigationResult:
        """取消当前 Action 并保留原目标，供 resume() 重新下发。

        Nav2 Humble 没有原生导航暂停服务，因此暂停的底层终态为
        CANCELED；多点导航恢复时会重新下发原始完整点位序列。
        """
        timeout = finite_float(timeout_sec, "timeout_sec")
        if timeout <= 0.0:
            raise ValidationError("timeout_sec must be greater than zero")
        if self._paused:
            assert self._pause_result is not None
            return self._pause_result
        if self._implementation.done:
            raise NavigationError("cannot pause a finished navigation goal")
        self.feedback
        result = self._implementation.cancel(timeout)
        if result.status != "CANCELED":
            raise NavigationError(
                "navigation could not pause; final status is "
                f"{result.status}"
            )
        self._paused = True
        self._pause_result = result
        return result

    def resume(
        self, *, server_timeout_sec: float = 5.0
    ) -> "NavigationHandle":
        """重新下发 pause() 保存的原目标并复用当前公共句柄。"""
        timeout = finite_float(server_timeout_sec, "server_timeout_sec")
        if timeout <= 0.0:
            raise ValidationError(
                "server_timeout_sec must be greater than zero"
            )
        if not self._paused:
            raise NavigationError("navigation goal is not paused")
        implementation = self._restart(timeout)
        self._implementation = implementation
        self._paused = False
        self._pause_result = None
        return self

    def cancel(self, *, timeout_sec: float = 5.0) -> NavigationResult:
        """取消活动目标，或把已暂停任务标记为最终取消。"""
        timeout = finite_float(timeout_sec, "timeout_sec")
        if timeout <= 0.0:
            raise ValidationError("timeout_sec must be greater than zero")
        if self._paused:
            self._paused = False
            assert self._pause_result is not None
            return self._pause_result
        return self._implementation.cancel(timeout)

    @staticmethod
    def _require_success(result: NavigationResult) -> NavigationResult:
        """返回成功结果，并把取消或失败状态转换为统一异常。"""
        if result.succeeded:
            return result
        if result.status == "CANCELED":
            raise NavigationCancelledError("navigation goal was cancelled")
        raise NavigationError(f"navigation ended with status {result.status}")


class Navigation:
    """提供导航下发、清图、当前位姿和活动目标管理。"""

    def __init__(self, backend: RobotBackend) -> None:
        """使用指定机器人后端创建导航控制器。"""
        self._backend = backend
        self._active: List[NavigationHandle] = []

    @staticmethod
    def _mode(value: NavigationMode) -> NavigationMode:
        """将枚举或枚举值转换为有效导航模式。"""
        if isinstance(value, NavigationMode):
            return value
        try:
            return NavigationMode(value)
        except (TypeError, ValueError) as exc:
            raise ValidationError("invalid NavigationMode") from exc

    def navigate_to(
        self,
        pose: NavigationPose,
        *,
        mode: NavigationMode = NavigationMode.DEFAULT,
        behavior_tree: str = "",
        server_timeout_sec: float = 5.0,
    ) -> NavigationHandle:
        """向 Nav2 非阻塞下发单个平面导航目标。

        Args:
            pose: 米和弧度制的导航目标。
            mode: 默认或精确导航策略。
            behavior_tree: 非空时覆盖 mode 对应行为树。
            server_timeout_sec: 等待 Action Server 确认的秒数。
        Returns:
            可暂停、恢复、等待和取消的导航句柄。
        """
        if not isinstance(pose, NavigationPose):
            raise ValidationError("pose must be a NavigationPose")
        timeout = finite_float(server_timeout_sec, "server_timeout_sec")
        if timeout <= 0.0:
            raise ValidationError(
                "server_timeout_sec must be greater than zero"
            )
        target = pose.validated()
        selected_mode = self._mode(mode)
        selected_tree = str(behavior_tree)

        def start(selected_timeout: float) -> BackendNavigationHandle:
            """向后端下发保存的单点目标。"""
            return self._backend.navigate_to(
                target, selected_mode, selected_tree, selected_timeout
            )

        handle = NavigationHandle(start(timeout), start)
        self._active.append(handle)
        return handle

    def navigate_through(
        self,
        poses: Sequence[NavigationPose],
        *,
        mode: NavigationMode = NavigationMode.DEFAULT,
        behavior_tree: str = "",
        server_timeout_sec: float = 5.0,
    ) -> NavigationHandle:
        """向 Nav2 非阻塞下发按顺序执行的多个目标。

        Args:
            poses: 非空 NavigationPose 序列。
            mode: 默认或精确导航策略。
            behavior_tree: 非空时覆盖 mode 对应行为树。
            server_timeout_sec: 等待 Action Server 确认的秒数。
        Returns:
            可暂停、恢复、等待和取消的导航句柄。
        """
        try:
            pose_items = tuple(poses)
        except TypeError as exc:
            raise ValidationError("poses must be a sequence") from exc
        if not pose_items:
            raise ValidationError("poses must not be empty")
        if any(not isinstance(pose, NavigationPose) for pose in pose_items):
            raise ValidationError("every pose must be a NavigationPose")
        timeout = finite_float(server_timeout_sec, "server_timeout_sec")
        if timeout <= 0.0:
            raise ValidationError(
                "server_timeout_sec must be greater than zero"
            )
        targets = tuple(pose.validated() for pose in pose_items)
        selected_mode = self._mode(mode)
        selected_tree = str(behavior_tree)

        def start(selected_timeout: float) -> BackendNavigationHandle:
            """向后端重新下发原始完整多点序列。"""
            return self._backend.navigate_through(
                targets, selected_mode, selected_tree, selected_timeout
            )

        handle = NavigationHandle(start(timeout), start)
        self._active.append(handle)
        return handle

    def navigate(
        self,
        poses: Sequence[NavigationPose],
        *,
        mode: NavigationMode = NavigationMode.DEFAULT,
        behavior_tree: str = "",
        server_timeout_sec: float = 5.0,
    ) -> NavigationHandle:
        """接收导航点序列并自动选择单点或多点导航接口。

        Args:
            poses: 一个或多个按执行顺序排列的 NavigationPose。
            mode: 默认或精确导航策略。
            behavior_tree: 非空时覆盖 mode 对应行为树。
            server_timeout_sec: 等待 Action Server 接受目标的秒数。
        Returns:
            可等待、暂停、恢复和取消的统一导航句柄。
        Raises:
            ValidationError: 序列为空、目标类型或参数不合法。
        """
        try:
            targets = tuple(poses)
        except TypeError as exc:
            raise ValidationError("poses must be a sequence") from exc
        if not targets:
            raise ValidationError("poses must contain at least one target")
        if len(targets) == 1:
            return self.navigate_to(
                targets[0],
                mode=mode,
                behavior_tree=behavior_tree,
                server_timeout_sec=server_timeout_sec,
            )
        return self.navigate_through(
            targets,
            mode=mode,
            behavior_tree=behavior_tree,
            server_timeout_sec=server_timeout_sec,
        )

    def navigate_waypoints(
        self,
        waypoint_ids: Sequence[int],
        *,
        path: Optional[Union[str, Path]] = None,
        mode: NavigationMode = NavigationMode.DEFAULT,
        behavior_tree: str = "",
        server_timeout_sec: float = 5.0,
    ) -> NavigationHandle:
        """从点位文件选择编号序列并执行单点或多点导航。

        Args:
            waypoint_ids: 一个或多个按执行顺序排列的点位编号。
            path: 可选点位文件；None 使用运行时默认路径。
            mode: 默认或精确导航策略。
            behavior_tree: 非空时覆盖 mode 对应行为树。
            server_timeout_sec: 等待 Action Server 接受目标的秒数。
        Returns:
            由 navigate() 创建的统一导航句柄。
        Raises:
            ValidationError: 编号序列为空、点位不存在或文件不合法。
            OSError: 点位文件无法读取。
        """
        from .waypoints import load_waypoints

        try:
            identifiers = tuple(waypoint_ids)
        except TypeError as exc:
            raise ValidationError("waypoint_ids must be a sequence") from exc
        if not identifiers:
            raise ValidationError(
                "waypoint_ids must contain at least one identifier"
            )
        if any(
            isinstance(item, bool) or not isinstance(item, int) or item < 1
            for item in identifiers
        ):
            raise ValidationError(
                "every waypoint id must be a positive integer"
            )
        waypoints = load_waypoints(path)
        missing = [item for item in identifiers if item not in waypoints]
        if missing:
            values = ",".join(str(item) for item in missing)
            raise ValidationError(f"unknown waypoint ids: {values}")
        return self.navigate(
            [waypoints[item].pose for item in identifiers],
            mode=mode,
            behavior_tree=behavior_tree,
            server_timeout_sec=server_timeout_sec,
        )

    def clear_local_costmap(
        self, *, timeout_sec: float = 5.0
    ) -> CommandResult:
        """清除 Nav2 local_costmap 的全部代价值。"""
        timeout = finite_float(timeout_sec, "timeout_sec")
        if timeout <= 0.0:
            raise ValidationError("timeout_sec must be greater than zero")
        return self._backend.clear_local_costmap(timeout)

    def clear_global_costmap(
        self, *, timeout_sec: float = 5.0
    ) -> CommandResult:
        """清除 Nav2 global_costmap 的全部代价值。"""
        timeout = finite_float(timeout_sec, "timeout_sec")
        if timeout <= 0.0:
            raise ValidationError("timeout_sec must be greater than zero")
        return self._backend.clear_global_costmap(timeout)

    def current_pose(
        self,
        *,
        global_frame: str = "map",
        base_frame: str = "base_link",
        timeout_sec: float = 2.0,
    ) -> NavigationPose:
        """读取 global_frame 到 base_frame 的当前平面 TF 位姿。"""
        if not isinstance(global_frame, str) or not global_frame.strip():
            raise ValidationError("global_frame must be a non-empty string")
        if not isinstance(base_frame, str) or not base_frame.strip():
            raise ValidationError("base_frame must be a non-empty string")
        timeout = finite_float(timeout_sec, "timeout_sec")
        if timeout <= 0.0:
            raise ValidationError("timeout_sec must be greater than zero")
        return self._backend.get_navigation_pose(
            global_frame.strip(), base_frame.strip(), timeout
        ).validated()

    def cancel_all(self, *, timeout_sec: float = 5.0) -> None:
        """尽力取消所有尚未结束的 SDK 导航目标并保留失败项。"""
        remaining: List[NavigationHandle] = []
        for handle in self._active:
            if handle.done:
                continue
            try:
                handle.cancel(timeout_sec=timeout_sec)
            except Exception:
                remaining.append(handle)
        self._active = remaining
