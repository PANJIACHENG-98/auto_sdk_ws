"""Plain-text waypoint storage shared by recorder and navigation examples."""

import csv
from dataclasses import dataclass
import os
from pathlib import Path
from typing import Dict, Optional, Union

from .errors import ValidationError
from .models import NavigationPose


WAYPOINT_FILE_ENV = "WHEELLOONG_AUTO_SDK_WAYPOINT_FILE"
WAYPOINT_HEADER = "# id,x_m,y_m,yaw_rad,frame_id"
PathLike = Union[str, Path]


def default_waypoint_path(
    *, fallback_path: Optional[PathLike] = None
) -> Path:
    """Resolve the portable default waypoint file.

    The environment variable takes priority. An optional existing fallback lets
    source examples select their project data. Installed code otherwise uses a
    source checkout file or the user's XDG data directory.
    """
    configured = os.environ.get(WAYPOINT_FILE_ENV, "").strip()
    if configured:
        return Path(configured).expanduser()
    if fallback_path is not None:
        fallback = Path(fallback_path).expanduser()
        if fallback.exists():
            return fallback
    source_file = (
        Path(__file__).resolve().parents[1]
        / "waypoint"
        / "waypoints.txt"
    )
    if source_file.exists():
        return source_file
    module_file = Path(__file__).absolute()
    for parent in module_file.parents:
        if parent.name != "install":
            continue
        colcon_source = (
            parent.parent
            / "src"
            / "wheelloong_auto_sdk"
            / "waypoint"
            / "waypoints.txt"
        )
        if colcon_source.exists():
            return colcon_source
    configured_data = os.environ.get("XDG_DATA_HOME", "").strip()
    data_home = (
        Path(configured_data).expanduser()
        if configured_data
        else Path.home() / ".local" / "share"
    )
    return data_home / "wheelloong_auto_sdk" / "waypoints.txt"


DEFAULT_WAYPOINT_PATH = default_waypoint_path()


def _resolved_path(path: Optional[PathLike]) -> Path:
    """Resolve an explicit path or the current configured default."""
    return default_waypoint_path() if path is None else Path(path).expanduser()


@dataclass(frozen=True)
class Waypoint:
    """Associate one positive integer identifier with a navigation pose."""

    waypoint_id: int
    pose: NavigationPose


def _validated_id(value: object) -> int:
    """Return a positive integer waypoint identifier."""
    if isinstance(value, bool):
        raise ValidationError("waypoint id must be a positive integer")
    try:
        waypoint_id = int(value)
    except (TypeError, ValueError) as exc:
        message = "waypoint id must be a positive integer"
        raise ValidationError(message) from exc
    if waypoint_id < 1 or str(value).strip() != str(waypoint_id):
        raise ValidationError("waypoint id must be a positive integer")
    return waypoint_id


def load_waypoints(
    path: Optional[PathLike] = None,
) -> Dict[int, Waypoint]:
    """Load comma-separated waypoints indexed by their integer identifiers.

    Args:
        path: TXT file containing rows; None uses the portable default.
    Returns:
        Insertion-ordered mapping from identifier to validated waypoint.
    Raises:
        ValidationError: A row is malformed or an identifier is duplicated.
    """
    source = _resolved_path(path)
    if not source.exists():
        return {}
    result: Dict[int, Waypoint] = {}
    with source.open("r", encoding="utf-8", newline="") as stream:
        for line_number, raw_line in enumerate(stream, start=1):
            stripped = raw_line.strip()
            if not stripped or stripped.startswith("#"):
                continue
            try:
                row = next(csv.reader([stripped]))
                if len(row) != 5:
                    raise ValueError("expected five columns")
                waypoint_id = _validated_id(row[0].strip())
                pose = NavigationPose(
                    float(row[1]),
                    float(row[2]),
                    float(row[3]),
                    frame_id=row[4].strip() or "map",
                ).validated()
            except (TypeError, ValueError, ValidationError) as exc:
                message = f"invalid waypoint at {source}:{line_number}: {exc}"
                raise ValidationError(message) from exc
            if waypoint_id in result:
                message = (
                    f"duplicate waypoint id {waypoint_id} "
                    f"at {source}:{line_number}"
                )
                raise ValidationError(message)
            result[waypoint_id] = Waypoint(waypoint_id, pose)
    return result


def next_waypoint_id(path: Optional[PathLike] = None) -> int:
    """Return max(stored identifier)+1, starting at one."""
    waypoints = load_waypoints(path)
    return max(waypoints, default=0) + 1


def append_waypoint(
    pose: NavigationPose,
    *,
    waypoint_id: Optional[int] = None,
    path: Optional[PathLike] = None,
) -> Waypoint:
    """Append one validated waypoint and create the directory/file if needed.

    Args:
        pose: Map pose to persist.
        waypoint_id: Optional positive id; None allocates max(existing)+1.
        path: Destination TXT file; None uses the portable default.
    Returns:
        The exact waypoint written to disk.
    Raises:
        ValidationError: Pose/id is invalid or the id already exists.
    """
    if not isinstance(pose, NavigationPose):
        raise ValidationError("pose must be a NavigationPose")
    target = _resolved_path(path)
    existing = load_waypoints(target)
    selected_id = (
        max(existing, default=0) + 1
        if waypoint_id is None
        else _validated_id(waypoint_id)
    )
    if selected_id in existing:
        raise ValidationError(f"waypoint id {selected_id} already exists")
    validated_pose = pose.validated()
    target.parent.mkdir(parents=True, exist_ok=True)
    needs_header = not target.exists() or target.stat().st_size == 0
    with target.open("a", encoding="utf-8", newline="") as stream:
        if needs_header:
            stream.write(WAYPOINT_HEADER + "\n")
        writer = csv.writer(stream, lineterminator="\n")
        writer.writerow(
            (
                selected_id,
                f"{validated_pose.x_m:.9f}",
                f"{validated_pose.y_m:.9f}",
                f"{validated_pose.yaw_rad:.9f}",
                validated_pose.frame_id,
            )
        )
    return Waypoint(selected_id, validated_pose)
