"""Direct ROS 2 backend for the installed Wheelloong/Shiloong interfaces."""

from dataclasses import replace
import math
import os
import threading
import time
from typing import Optional, Sequence

from action_msgs.msg import GoalStatus
from arm_common.msg import Speed
from arm_common.srv import HoldOn, MoveJ, MoveL, MoveP
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateThroughPoses, NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap
import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.context import Context
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from system_common.msg import SystemInfo
from system_common.srv import Enable, GripperCtrl, JntsCtrl, SwCtrlMode
from tf2_ros import Buffer, TransformException, TransformListener
from voice_common.srv import PlayAudio, SpeakText

from .base import BackendNavigationHandle, JointTuple, RobotBackend
from ..errors import (
    BackendError,
    CommandTimeoutError,
    NavigationError,
    NavigationRejectedError,
    RobotCommandError,
    RobotStateError,
    ServiceUnavailableError,
    ValidationError,
)
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


STATUS_NAMES = {
    GoalStatus.STATUS_UNKNOWN: "UNKNOWN",
    GoalStatus.STATUS_ACCEPTED: "ACCEPTED",
    GoalStatus.STATUS_EXECUTING: "EXECUTING",
    GoalStatus.STATUS_CANCELING: "CANCELING",
    GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
    GoalStatus.STATUS_CANCELED: "CANCELED",
    GoalStatus.STATUS_ABORTED: "ABORTED",
}


class _StandaloneRuntime:
    """拥有私有 ROS Context、Node、执行器及其后台线程。"""

    def __init__(self, node_name: str, ros_args: Optional[Sequence[str]]) -> None:
        """初始化私有 ROS 运行时并启动两线程执行器。"""
        self.context = Context()
        self.context.init(args=list(ros_args) if ros_args is not None else None)
        self.node = Node(node_name, context=self.context)
        self.executor = MultiThreadedExecutor(num_threads=2, context=self.context)
        self.executor.add_node(self.node)
        self.thread = threading.Thread(
            target=self.executor.spin,
            name=f"{node_name}_executor",
            daemon=True,
        )
        self.thread.start()
        self.closed = False
        self.executor_stopped = False

    def stop_executor(self) -> None:
        """幂等停止执行器并等待后台 spin 线程退出。"""
        if self.executor_stopped:
            return
        self.executor_stopped = True
        self.executor.shutdown(timeout_sec=2.0)
        self.thread.join(timeout=2.0)

    def close(self) -> None:
        """停止执行器、销毁私有 Node 并关闭 Context。"""
        if self.closed:
            return
        self.closed = True
        self.stop_executor()
        self.node.destroy_node()
        if self.context.ok():
            self.context.shutdown()


def _duration_sec(value) -> float:
    """将 ROS Duration 消息转换为浮点秒数。"""
    return float(value.sec) + float(value.nanosec) / 1_000_000_000.0


class _RosNavigationHandle(BackendNavigationHandle):
    """把 rclpy ClientGoalHandle 转换为后端无关导航句柄。"""

    def __init__(self, backend: "ROS2Backend", goal_handle, action_name: str) -> None:
        """保存目标句柄并立即创建异步结果请求。"""
        self._backend = backend
        self._goal_handle = goal_handle
        self._action_name = action_name
        self._result_future = goal_handle.get_result_async()
        self._feedback: Optional[NavigationFeedback] = None
        self._lock = threading.Lock()
        self._cached_result: Optional[NavigationResult] = None

    def update_feedback(self, feedback) -> None:
        """将 Nav2 反馈转换为公共反馈模型并线程安全保存。"""
        poses_remaining = getattr(feedback, "number_of_poses_remaining", None)
        converted = NavigationFeedback(
            distance_remaining_m=float(feedback.distance_remaining),
            estimated_time_remaining_sec=_duration_sec(feedback.estimated_time_remaining),
            navigation_time_sec=_duration_sec(feedback.navigation_time),
            recoveries=int(feedback.number_of_recoveries),
            poses_remaining=None if poses_remaining is None else int(poses_remaining),
        )
        with self._lock:
            self._feedback = converted

    @property
    def feedback(self) -> Optional[NavigationFeedback]:
        """线程安全返回最近一次导航反馈。"""
        with self._lock:
            return self._feedback

    @property
    def done(self) -> bool:
        """判断结果是否已缓存或 ROS Future 已完成。"""
        return self._cached_result is not None or self._result_future.done()

    def _convert_result(self, response) -> NavigationResult:
        """将 Nav2 结果状态码转换并缓存为公共结果。"""
        status = int(response.status)
        result = NavigationResult(
            status=STATUS_NAMES.get(status, f"STATUS_{status}"),
            succeeded=status == GoalStatus.STATUS_SUCCEEDED,
        )
        self._cached_result = result
        return result

    def wait(self, timeout_sec: Optional[float]) -> NavigationResult:
        """等待结果 Future 完成并返回转换后的导航结果。"""
        if self._cached_result is not None:
            return self._cached_result
        response = self._backend._wait_future(
            self._result_future,
            timeout_sec,
            f"{self._action_name} result",
        )
        return self._convert_result(response)

    def cancel(self, timeout_sec: float) -> NavigationResult:
        """请求取消目标并在同一截止时间内等待最终状态。"""
        if self._cached_result is not None:
            return self._cached_result
        deadline = time.monotonic() + timeout_sec
        if self._result_future.done():
            return self.wait(0.0)
        cancel_response = self._backend._wait_future(
            self._goal_handle.cancel_goal_async(),
            timeout_sec,
            f"{self._action_name} cancellation acknowledgement",
        )
        remaining = deadline - time.monotonic()
        if not cancel_response.goals_canceling and not self._result_future.done():
            raise NavigationError(f"{self._action_name} cancellation was not accepted")
        if remaining <= 0.0 and not self._result_future.done():
            raise CommandTimeoutError(
                f"timed out waiting for {self._action_name} cancellation to finish"
            )
        return self.wait(max(0.0, remaining))


class ROS2Backend(RobotBackend):
    """将稳定语义接口翻译为现有 ROS 2 Topic、Service 和 Action。"""

    SYSTEM_INFO_TOPIC = "/system/get_info"

    def __init__(
        self, node: Node, *, runtime: Optional[_StandaloneRuntime] = None
    ) -> None:
        """在指定 Node 上创建所有订阅、服务客户端和 Action 客户端。"""
        if not isinstance(node, Node):
            raise TypeError("node must be an rclpy.node.Node")
        self._node = node
        self._runtime = runtime
        self._closed = False
        self._callback_group = ReentrantCallbackGroup()
        self._state_condition = threading.Condition()
        self._state: Optional[SystemState] = None
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, node)

        self._subscriptions = [
            node.create_subscription(
                SystemInfo,
                self.SYSTEM_INFO_TOPIC,
                self._on_system_info,
                10,
                callback_group=self._callback_group,
            )
        ]
        self._clients = {
            "set_control_mode": node.create_client(
                SwCtrlMode, "/system/set_ctrl_mode", callback_group=self._callback_group
            ),
            "enable": node.create_client(
                Enable, "/system/enable", callback_group=self._callback_group
            ),
            "gripper": node.create_client(
                GripperCtrl, "/system/gripper_ctrl", callback_group=self._callback_group
            ),
            "body": node.create_client(
                JntsCtrl, "/system/jnts_ctrl", callback_group=self._callback_group
            ),
            "arm_hold": node.create_client(
                HoldOn, "/arm_driver/hold_on", callback_group=self._callback_group
            ),
            "move_j": node.create_client(
                MoveJ, "/arm_driver/joint_move", callback_group=self._callback_group
            ),
            "move_l": node.create_client(
                MoveL, "/arm_driver/linear_move", callback_group=self._callback_group
            ),
            "move_p": node.create_client(
                MoveP, "/arm_driver/point_move", callback_group=self._callback_group
            ),
            "speak": node.create_client(
                SpeakText, "/voice/speak_text", callback_group=self._callback_group
            ),
            "play_audio": node.create_client(
                PlayAudio, "/voice/play_audio", callback_group=self._callback_group
            ),
            "clear_local_costmap": node.create_client(
                ClearEntireCostmap,
                "/local_costmap/clear_entirely_local_costmap",
                callback_group=self._callback_group,
            ),
            "clear_global_costmap": node.create_client(
                ClearEntireCostmap,
                "/global_costmap/clear_entirely_global_costmap",
                callback_group=self._callback_group,
            ),
        }
        self._navigate_to_client = ActionClient(
            node,
            NavigateToPose,
            "/navigate_to_pose",
            callback_group=self._callback_group,
        )
        self._navigate_through_client = ActionClient(
            node,
            NavigateThroughPoses,
            "/navigate_through_poses",
            callback_group=self._callback_group,
        )

    @classmethod
    def standalone(
        cls,
        node_name: str = "wheelloong_auto_sdk",
        ros_args: Optional[Sequence[str]] = None,
    ) -> "ROS2Backend":
        """创建拥有私有 ROS 运行时的直连后端。"""
        runtime = _StandaloneRuntime(node_name, ros_args)
        try:
            return cls(runtime.node, runtime=runtime)
        except Exception:
            runtime.close()
            raise

    @property
    def node(self) -> Node:
        """返回当前后端使用的 rclpy Node。"""
        return self._node

    def close(self) -> None:
        """按照运行时所有权规则幂等关闭后端。"""
        if self._closed:
            return
        self._closed = True
        # Humble ActionClient.destroy() races an executor that is currently
        # rebuilding its wait set.  We can stop our private executor safely.
        # For a borrowed Node, its owner keeps entity ownership and destroys
        # everything with the Node after stopping its own executor.
        if self._runtime is None:
            return
        self._runtime.stop_executor()
        self._tf_listener.unregister()
        self._navigate_to_client.destroy()
        self._navigate_through_client.destroy()
        for client in self._clients.values():
            self._node.destroy_client(client)
        for subscription in self._subscriptions:
            self._node.destroy_subscription(subscription)
        self._runtime.close()

    def _wait_future(self, future, timeout_sec: Optional[float], description: str):
        """等待外部执行器推进 Future，并统一转换超时和执行异常。"""
        deadline = None if timeout_sec is None else time.monotonic() + timeout_sec
        while not future.done():
            if not self._node.context.ok():
                raise BackendError(f"ROS context stopped while waiting for {description}")
            if deadline is not None and time.monotonic() >= deadline:
                executor_hint = ""
                if self._runtime is None:
                    executor_hint = (
                        "; an externally supplied Node must already be spinning in a "
                        "MultiThreadedExecutor when synchronous SDK methods are called"
                    )
                raise CommandTimeoutError(f"timed out waiting for {description}{executor_hint}")
            time.sleep(0.01)
        try:
            return future.result()
        except Exception as exc:
            raise BackendError(f"{description} raised: {exc}") from exc

    def _service_call(self, key: str, request, operation: str, timeout_sec: float):
        """等待服务、异步调用并把错误码转换为统一结果或异常。"""
        client = self._clients[key]
        started = time.monotonic()
        if not client.wait_for_service(timeout_sec=timeout_sec):
            raise ServiceUnavailableError(f"service for {operation} is unavailable")
        remaining = max(0.0, timeout_sec - (time.monotonic() - started))
        response = self._wait_future(
            client.call_async(request), remaining, f"{operation} response"
        )
        code = int(getattr(response, "error_code", 0))
        message = str(getattr(response, "error_msg", ""))
        if code != 0:
            raise RobotCommandError(operation, code, message)
        return CommandResult(operation, code, message)

    @staticmethod
    def _format_error_pair(pair) -> str:
        """将 ErrorPair 消息格式化为便于展示的文本。"""
        return f"{int(pair.error_code)}: {str(pair.error_msg)}"

    @staticmethod
    def _system_pose(values, frame_id: str) -> Optional[CartesianPoseMM]:
        """把 SystemInfo 的 xyz+xyzw 转为有效 TCP 位姿。

        无效或尚未初始化的全零四元数返回 None，避免状态回调中断。
        """
        try:
            return CartesianPoseMM(*values, frame_id=frame_id).validated()
        except (TypeError, ValueError, ValidationError):
            return None

    def _on_system_info(self, msg: SystemInfo) -> None:
        """把 SystemInfo 消息转换为状态快照并唤醒等待者。"""
        arm_frame = str(msg.arm_base_used or "arm_driver")
        state = SystemState(
            received_monotonic=time.monotonic(),
            heartbeat=int(msg.heartbeat),
            control_mode=int(msg.current_control_mode),
            errors=tuple(self._format_error_pair(item) for item in msg.error_map),
            warnings=tuple(self._format_error_pair(item) for item in msg.warning_map),
            left_arm_enabled=bool(msg.left_arm_enabled),
            right_arm_enabled=bool(msg.right_arm_enabled),
            head_pitch_enabled=bool(msg.neck_rotate_enabled),
            head_yaw_enabled=bool(msg.neck_spin_enabled),
            waist_pitch_enabled=bool(msg.lumbar_rotate_enabled),
            waist_lift_enabled=bool(msg.lumbar_lift_enabled),
            left_arm_joints=tuple(float(value) for value in msg.arm_jnts_left),
            right_arm_joints=tuple(float(value) for value in msg.arm_jnts_right),
            left_arm_pose=self._system_pose(msg.arm_quat_left, arm_frame),
            right_arm_pose=self._system_pose(msg.arm_quat_right, arm_frame),
            head_pitch_rad=float(msg.neck[0]),
            head_yaw_rad=float(msg.neck[1]),
            waist_pitch_rad=float(msg.lumbar[0]),
            waist_lift_mm=float(msg.lumbar[1]),
            left_gripper=float(msg.gripper_pose[0]),
            right_gripper=float(msg.gripper_pose[1]),
        )
        with self._state_condition:
            self._state = state
            self._state_condition.notify_all()

    def get_system_state(self, max_age_sec: float, wait_timeout_sec: float) -> SystemState:
        """等待并返回年龄不超过指定阈值的系统状态。"""
        deadline = time.monotonic() + wait_timeout_sec
        with self._state_condition:
            while self._state is None or self._state.age_sec > max_age_sec:
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    raise RobotStateError(
                        f"no fresh {self.SYSTEM_INFO_TOPIC} state within {wait_timeout_sec:.2f}s"
                    )
                self._state_condition.wait(timeout=min(remaining, 0.1))
            return self._state

    def wait_system_state(
        self,
        predicate: StatePredicate,
        timeout_sec: float,
        description: str,
    ) -> SystemState:
        """等待新鲜系统状态满足谓词，否则抛出超时。"""
        deadline = time.monotonic() + timeout_sec
        with self._state_condition:
            while True:
                if self._state is not None and self._state.age_sec <= 1.0:
                    if predicate(self._state):
                        return self._state
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    raise CommandTimeoutError(
                        f"timed out waiting for robot state: {description}"
                    )
                self._state_condition.wait(timeout=min(remaining, 0.1))

    def set_control_mode(self, mode: int, timeout_sec: float) -> CommandResult:
        """调用系统服务设置控制模式。"""
        request = SwCtrlMode.Request()
        request.ctrl_mode = int(mode)
        return self._service_call(
            "set_control_mode", request, "set control mode", timeout_sec
        )

    def set_enabled(self, axes: AxisSelection, timeout_sec: float) -> CommandResult:
        """把轴选择映射到 Enable 请求并下发。"""
        request = Enable.Request()
        request.left_arm = axes.left_arm
        request.right_arm = axes.right_arm
        request.neck_rotate = axes.head_pitch
        request.neck_spin = axes.head_yaw
        request.lumbar_rotate = axes.waist_pitch
        request.lumbar_lift = axes.waist_lift
        return self._service_call("enable", request, "set axis enable state", timeout_sec)

    def arm_hold(self, timeout_sec: float) -> CommandResult:
        """调用机械臂 Hold 服务停止双臂运动。"""
        return self._service_call(
            "arm_hold", HoldOn.Request(), "hold arms", timeout_sec
        )

    @staticmethod
    def _robot_index(left, right) -> int:
        """根据左右目标是否存在返回左、右或双臂索引。"""
        if left is not None and right is not None:
            return -1
        return 0 if left is not None else 1

    @staticmethod
    def _speed_message(move: str, speed: float, acceleration: float) -> Speed:
        """把语义速度和加速度填入对应运动类型字段。"""
        value = Speed()
        if move == "j":
            value.move_j_vel = speed
            value.move_j_acc = acceleration
        elif move == "l":
            value.move_l_vel = speed
            value.move_l_acc = acceleration
        else:
            value.move_p_vel = speed
            value.move_p_acc = acceleration
        return value

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
        """构造并调用底层 MoveJ 服务。"""
        request = MoveJ.Request()
        request.robot_index = self._robot_index(left, right)
        request.non_blocking = not wait
        request.mode_left = int(mode)
        request.mode_right = int(mode)
        if left is not None:
            request.joint_left = list(left)
        if right is not None:
            request.joint_right = list(right)
        request.speed_left = self._speed_message("j", speed_rad_s, acceleration_rad_s2)
        request.speed_right = self._speed_message("j", speed_rad_s, acceleration_rad_s2)
        # 与正式 auto_client.py 一致：预留力矩数组和 zone 显式使用零值。
        request.effort_left = [0.0] * 7
        request.effort_right = [0.0] * 7
        request.zone_left = 0
        request.zone_right = 0
        return self._service_call("move_j", request, "MoveJ", timeout_sec)

    def _pose_message(self, pose: CartesianPoseMM) -> PoseStamped:
        """把毫米制公共法兰位姿转换为 PoseStamped。"""
        result = PoseStamped()
        result.header.frame_id = pose.frame_id
        result.header.stamp = self._node.get_clock().now().to_msg()
        result.pose.position.x = pose.x_mm
        result.pose.position.y = pose.y_mm
        result.pose.position.z = pose.z_mm
        result.pose.orientation.x = pose.qx
        result.pose.orientation.y = pose.qy
        result.pose.orientation.z = pose.qz
        result.pose.orientation.w = pose.qw
        return result

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
        """构造并调用底层 MoveL 服务。"""
        request = MoveL.Request()
        request.robot_index = self._robot_index(left, right)
        request.non_blocking = not wait
        request.mode_left = int(mode)
        request.mode_right = int(mode)
        if left is not None:
            request.pose_left = self._pose_message(left)
        if right is not None:
            request.pose_right = self._pose_message(right)
        request.psi_left = left_psi_rad
        request.psi_right = right_psi_rad
        request.speed_left = self._speed_message("l", speed_mm_s, acceleration_mm_s2)
        request.speed_right = self._speed_message("l", speed_mm_s, acceleration_mm_s2)
        # SDK 暂不开放平滑区参数，明确采用接口默认值 0。
        request.zone_left = 0
        request.zone_right = 0
        return self._service_call("move_l", request, "MoveL", timeout_sec)

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
        """构造并调用底层 MoveP 服务。"""
        request = MoveP.Request()
        request.robot_index = self._robot_index(left, right)
        request.non_blocking = not wait
        if left is not None:
            request.pose_left = self._pose_message(left)
        if right is not None:
            request.pose_right = self._pose_message(right)
        if left_reference is not None:
            request.ref_joint_left = list(left_reference)
        if right_reference is not None:
            request.ref_joint_right = list(right_reference)
        request.speed_left = self._speed_message("p", speed_rad_s, acceleration_rad_s2)
        request.speed_right = self._speed_message("p", speed_rad_s, acceleration_rad_s2)
        # SDK 暂不开放平滑区参数，明确采用接口默认值 0。
        request.zone_left = 0
        request.zone_right = 0
        return self._service_call("move_p", request, "MoveP", timeout_sec)

    @staticmethod
    def _body_request(
        values: Sequence[Optional[float]],
        speed_percent: float,
        wait: bool,
        timeout_sec: float,
    ):
        """构造只选择非 None 轴的 JntsCtrl 请求。"""
        request = JntsCtrl.Request()
        request.index = [value is not None for value in values]
        request.value = [0.0 if value is None else float(value) for value in values]
        request.speed = speed_percent
        request.block = wait
        request.timeout_sec = timeout_sec
        return request

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
        """调用头腰服务，并在超时时尝试冻结已选择轴。"""
        values = (head_pitch_rad, head_yaw_rad, waist_pitch_rad, waist_lift_mm)
        request = self._body_request(values, speed_percent, wait, timeout_sec)
        try:
            local_timeout = timeout_sec + 1.0 if wait else timeout_sec
            return self._service_call("body", request, "move head/waist", local_timeout)
        except CommandTimeoutError:
            self._best_effort_freeze_body(values)
            raise

    def _best_effort_freeze_body(self, commanded_values) -> None:
        """用最新实际头腰位置非阻塞覆盖超时后仍活动的目标。"""
        try:
            state = self.get_system_state(1.0, 0.2)
            current = (
                state.head_pitch_rad,
                state.head_yaw_rad,
                state.waist_pitch_rad,
                state.waist_lift_mm,
            )
            values = tuple(
                current[index] if value is not None else None
                for index, value in enumerate(commanded_values)
            )
            self._clients["body"].call_async(
                self._body_request(values, 50.0, False, 1.0)
            )
        except Exception:
            pass

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
        """调用夹爪服务并可等待 SystemInfo 实际位置到达。"""
        started = time.monotonic()
        request = GripperCtrl.Request()
        request.index = side_index
        request.value = position
        request.speed = speed
        request.torque = torque
        result = self._service_call(
            "gripper", request, "set gripper", timeout_sec
        )
        if not wait:
            return result
        remaining = timeout_sec - (time.monotonic() - started)
        if remaining <= 0.0:
            raise CommandTimeoutError("gripper service returned after the wait deadline")

        def reached(state: SystemState) -> bool:
            """判断所选夹爪是否均已进入目标容差。"""
            left_ok = abs(state.left_gripper - position) <= tolerance
            right_ok = abs(state.right_gripper - position) <= tolerance
            return (
                (side_index == 0 and left_ok)
                or (side_index == 1 and right_ok)
                or (side_index == -1 and left_ok and right_ok)
            )

        self.wait_system_state(reached, remaining, "gripper target")
        return result

    def speak(
        self, text: str, wait: bool, speech_timeout_sec: float, call_timeout_sec: float
    ) -> CommandResult:
        """构造并调用文本播报服务。"""
        request = SpeakText.Request()
        request.text = text
        request.wait_until_done = wait
        request.timeout_sec = speech_timeout_sec
        return self._service_call("speak", request, "speak text", call_timeout_sec)

    def play_audio(
        self, file_name: str, play_count: int, wait: bool, timeout_sec: float
    ) -> CommandResult:
        """构造并调用本地音频播放服务。"""
        request = PlayAudio.Request()
        request.file_name = file_name
        request.play_count = play_count
        request.block = wait
        return self._service_call("play_audio", request, "play audio", timeout_sec)

    def _navigation_pose_message(self, pose: NavigationPose) -> PoseStamped:
        """把平面米制导航目标转换为带偏航四元数的 PoseStamped。"""
        result = PoseStamped()
        result.header.frame_id = pose.frame_id
        result.header.stamp = self._node.get_clock().now().to_msg()
        result.pose.position.x = pose.x_m
        result.pose.position.y = pose.y_m
        result.pose.position.z = pose.z_m
        result.pose.orientation.z = math.sin(pose.yaw_rad * 0.5)
        result.pose.orientation.w = math.cos(pose.yaw_rad * 0.5)
        return result

    @staticmethod
    def _behavior_tree(
        mode: NavigationMode, behavior_tree: str, *, through: bool
    ) -> str:
        """根据导航模式、环境变量和显式值解析行为树路径。"""
        if behavior_tree.strip():
            return behavior_tree.strip()
        prefix = os.environ.get("ROBOT_NAV_VERSION", "").strip()
        if through:
            if prefix in ("weloong_nav", "sloong_nav"):
                return f"{prefix}/navigate_through_poses.xml"
            # WLOONG 等产品型号不是行为树目录；交给 Nav2 使用启动配置。
            return ""
        suffixes = {
            NavigationMode.DEFAULT: "",
            NavigationMode.PRECISE_BACKWARD: "navigate_to_goal_precise_backward.xml",
            NavigationMode.PRECISE_FORWARD: "navigate_to_goal_precise_forward.xml",
            NavigationMode.PRECISE: "navigate_to_goal_precise.xml",
        }
        suffix = suffixes[mode]
        if not suffix:
            return ""
        if prefix not in ("weloong_nav", "sloong_nav"):
            raise BackendError(
                "ROBOT_NAV_VERSION must be weloong_nav or sloong_nav for precise navigation"
            )
        return f"{prefix}/{suffix}"

    def _send_navigation_goal(
        self,
        action_client,
        goal,
        action_name: str,
        server_timeout_sec: float,
    ) -> BackendNavigationHandle:
        """等待 Action Server、发送目标并返回已接受的导航句柄。"""
        if not action_client.wait_for_server(timeout_sec=server_timeout_sec):
            raise ServiceUnavailableError(f"{action_name} action server is unavailable")
        holder = {}

        def feedback_callback(message) -> None:
            """把 Action 反馈转发给完成创建后的公共句柄。"""
            handle = holder.get("handle")
            if handle is not None:
                handle.update_feedback(message.feedback)

        goal_future = action_client.send_goal_async(
            goal, feedback_callback=feedback_callback
        )
        goal_handle = self._wait_future(
            goal_future, server_timeout_sec, f"{action_name} goal acknowledgement"
        )
        if not goal_handle.accepted:
            raise NavigationRejectedError(f"{action_name} goal was rejected")
        handle = _RosNavigationHandle(self, goal_handle, action_name)
        holder["handle"] = handle
        return handle

    def navigate_to(
        self,
        pose: NavigationPose,
        mode: NavigationMode,
        behavior_tree: str,
        server_timeout_sec: float,
    ) -> BackendNavigationHandle:
        """构造并下发 NavigateToPose 单点目标。"""
        goal = NavigateToPose.Goal()
        goal.pose = self._navigation_pose_message(pose)
        goal.behavior_tree = self._behavior_tree(mode, behavior_tree, through=False)
        return self._send_navigation_goal(
            self._navigate_to_client,
            goal,
            "NavigateToPose",
            server_timeout_sec,
        )

    def navigate_through(
        self,
        poses: Sequence[NavigationPose],
        mode: NavigationMode,
        behavior_tree: str,
        server_timeout_sec: float,
    ) -> BackendNavigationHandle:
        """构造并下发 NavigateThroughPoses 多点目标。"""
        goal = NavigateThroughPoses.Goal()
        goal.poses = [self._navigation_pose_message(pose) for pose in poses]
        goal.behavior_tree = self._behavior_tree(mode, behavior_tree, through=True)
        return self._send_navigation_goal(
            self._navigate_through_client,
            goal,
            "NavigateThroughPoses",
            server_timeout_sec,
        )

    def clear_local_costmap(self, timeout_sec: float) -> CommandResult:
        """调用 Nav2 服务清除完整局部代价地图。"""
        return self._service_call(
            "clear_local_costmap",
            ClearEntireCostmap.Request(),
            "clear local costmap",
            timeout_sec,
        )

    def clear_global_costmap(self, timeout_sec: float) -> CommandResult:
        """调用 Nav2 服务清除完整全局代价地图。"""
        return self._service_call(
            "clear_global_costmap",
            ClearEntireCostmap.Request(),
            "clear global costmap",
            timeout_sec,
        )

    @staticmethod
    def _navigation_pose_from_transform(
        transform, frame_id: str
    ) -> NavigationPose:
        """把全局到机器人基座 TF 转换为平面导航位姿。"""
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        sin_yaw = 2.0 * (
            rotation.w * rotation.z + rotation.x * rotation.y
        )
        cos_yaw = 1.0 - 2.0 * (
            rotation.y * rotation.y + rotation.z * rotation.z
        )
        return NavigationPose(
            x_m=translation.x,
            y_m=translation.y,
            yaw_rad=math.atan2(sin_yaw, cos_yaw),
            z_m=translation.z,
            frame_id=frame_id,
        ).validated()

    def get_navigation_pose(
        self,
        global_frame: str,
        base_frame: str,
        timeout_sec: float,
    ) -> NavigationPose:
        """读取 global_frame 到 base_frame 的最新 TF 位姿。"""
        try:
            transform = self._tf_buffer.lookup_transform(
                global_frame,
                base_frame,
                Time(),
                timeout=Duration(seconds=timeout_sec),
            )
        except TransformException as exc:
            raise RobotStateError(
                f"failed to get {global_frame}->{base_frame} pose: {exc}"
            ) from exc
        return self._navigation_pose_from_transform(transform, global_frame)
