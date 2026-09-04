"""Public exception hierarchy for the SDK."""

from typing import Optional


class WheelloongSdkError(RuntimeError):
    """Base class for all SDK errors."""


class ValidationError(WheelloongSdkError, ValueError):
    """A public API argument is invalid."""


class BackendError(WheelloongSdkError):
    """The selected backend failed independently of a robot command result."""


class ServiceUnavailableError(BackendError):
    """A required ROS service or action server is unavailable."""


class CommandTimeoutError(WheelloongSdkError, TimeoutError):
    """A command did not reach a terminal state before its deadline."""


class RobotStateError(WheelloongSdkError):
    """The robot state is stale, unsafe, or incompatible with the command."""


class RobotCommandError(WheelloongSdkError):
    """A robot service accepted the request but reported an error."""

    def __init__(
        self,
        operation: str,
        error_code: int,
        message: str,
        *,
        cause: Optional[BaseException] = None,
    ) -> None:
        """保存操作名、错误码和底层错误消息并生成统一异常文本。"""
        detail = message.strip() or "robot command failed"
        super().__init__(f"{operation} failed ({error_code}): {detail}")
        self.operation = operation
        self.error_code = int(error_code)
        self.robot_message = message
        self.__cause__ = cause


class NavigationError(WheelloongSdkError):
    """A navigation goal was rejected or ended unsuccessfully."""


class NavigationRejectedError(NavigationError):
    """The navigation action server rejected a goal."""


class NavigationCancelledError(NavigationError):
    """A navigation goal was cancelled before completion."""
