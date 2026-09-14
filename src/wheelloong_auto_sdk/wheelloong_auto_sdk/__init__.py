"""Stable Python API for Wheelloong/Shiloong AUTO-mode task programs."""

from .errors import (
    BackendError,
    CommandTimeoutError,
    NavigationCancelledError,
    NavigationError,
    NavigationRejectedError,
    RobotCommandError,
    RobotStateError,
    ServiceUnavailableError,
    ValidationError,
    WheelloongSdkError,
)
from .models import (
    ArmSide,
    AxisSelection,
    CartesianPoseMM,
    CommandResult,
    ControlMode,
    MotionMode,
    NavigationFeedback,
    NavigationMode,
    NavigationPose,
    NavigationResult,
    SystemState,
)
from .navigation import NavigationHandle
from .profiles import (
    RobotProfile,
    SHILOONG_PROFILE,
    WHEELLOONG_PROFILE,
    get_robot_profile,
)
from .robot import Robot
from .system import System
from .waypoints import (
    DEFAULT_WAYPOINT_PATH,
    WAYPOINT_FILE_ENV,
    Waypoint,
    append_waypoint,
    default_waypoint_path,
    load_waypoints,
    next_waypoint_id,
)

__version__ = "0.1.0"

__all__ = [
    "ArmSide",
    "AxisSelection",
    "BackendError",
    "CartesianPoseMM",
    "CommandResult",
    "CommandTimeoutError",
    "ControlMode",
    "DEFAULT_WAYPOINT_PATH",
    "MotionMode",
    "NavigationCancelledError",
    "NavigationError",
    "NavigationFeedback",
    "NavigationHandle",
    "NavigationMode",
    "NavigationPose",
    "NavigationRejectedError",
    "NavigationResult",
    "Robot",
    "RobotProfile",
    "RobotCommandError",
    "RobotStateError",
    "ServiceUnavailableError",
    "SHILOONG_PROFILE",
    "SystemState",
    "System",
    "ValidationError",
    "WAYPOINT_FILE_ENV",
    "Waypoint",
    "WheelloongSdkError",
    "WHEELLOONG_PROFILE",
    "append_waypoint",
    "default_waypoint_path",
    "get_robot_profile",
    "load_waypoints",
    "next_waypoint_id",
]
