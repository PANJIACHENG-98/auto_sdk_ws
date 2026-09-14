"""Validate waypoint persistence and direct SDK navigation calls."""

from contextlib import nullcontext
from dataclasses import replace
import importlib.util
from pathlib import Path
import sys
from unittest.mock import Mock

import pytest

from wheelloong_auto_sdk import (
    ControlMode,
    NavigationPose,
    Robot,
    ValidationError,
    WAYPOINT_FILE_ENV,
    append_waypoint,
    default_waypoint_path,
    load_waypoints,
    next_waypoint_id,
)
from wheelloong_auto_sdk.backend import MockBackend
import wheelloong_auto_sdk.waypoints as waypoint_storage


EXAMPLE_PATH = (
    Path(__file__).resolve().parents[1]
    / "examples"
    / "waypoint_navigation_demo.py"
)


def load_navigation_example():
    """Load the direct-call example without running its main function."""
    spec = importlib.util.spec_from_file_location(
        "waypoint_navigation_demo", EXAMPLE_PATH
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_waypoint_file_allocates_ids_and_preserves_pose_order(tmp_path):
    """Append numbered rows and load their validated navigation poses."""
    path = tmp_path / "waypoint" / "waypoints.txt"
    first = append_waypoint(NavigationPose(1.0, 2.0, 0.3), path=path)
    second = append_waypoint(
        NavigationPose(4.0, 5.0, -0.6), waypoint_id=3, path=path
    )

    assert first.waypoint_id == 1
    assert second.waypoint_id == 3
    loaded = load_waypoints(path)
    assert list(loaded) == [1, 3]
    assert loaded[1].pose == NavigationPose(1.0, 2.0, 0.3)
    assert loaded[3].pose == NavigationPose(4.0, 5.0, -0.6)
    assert next_waypoint_id(path) == 4


def test_waypoint_file_rejects_duplicate_or_malformed_rows(tmp_path):
    """Reject duplicate ids and rows that do not have five columns."""
    path = tmp_path / "waypoints.txt"
    append_waypoint(NavigationPose(0.0, 0.0, 0.0), path=path)
    with pytest.raises(ValidationError, match="already exists"):
        append_waypoint(
            NavigationPose(1.0, 1.0, 0.0), waypoint_id=1, path=path
        )
    path.write_text("1,2,3\n", encoding="utf-8")
    with pytest.raises(ValidationError, match="expected five columns"):
        load_waypoints(path)


def test_waypoint_default_path_uses_runtime_environment(tmp_path, monkeypatch):
    """Prefer a delivery-configured path without developer home hardcoding."""
    configured = tmp_path / "task_data" / "waypoints.txt"
    monkeypatch.setenv(WAYPOINT_FILE_ENV, str(configured))

    assert default_waypoint_path() == configured
    stored = append_waypoint(NavigationPose(1.0, 2.0, 0.3))

    assert stored.waypoint_id == 1
    assert load_waypoints()[1].pose == NavigationPose(1.0, 2.0, 0.3)


def test_waypoint_default_path_accepts_an_existing_source_fallback(
    tmp_path, monkeypatch
):
    """Let source examples override an installed package data fallback."""
    configured = tmp_path / "source" / "waypoint" / "waypoints.txt"
    configured.parent.mkdir(parents=True)
    configured.write_text("# test\n", encoding="utf-8")
    monkeypatch.delenv(WAYPOINT_FILE_ENV, raising=False)

    assert default_waypoint_path(fallback_path=configured) == configured


def test_installed_sdk_finds_its_colcon_source_waypoint_file(
    tmp_path, monkeypatch
):
    """Resolve workspace source data from a colcon install module path."""
    workspace = tmp_path / "auto_sdk_ws"
    configured = (
        workspace
        / "src"
        / "wheelloong_auto_sdk"
        / "waypoint"
        / "waypoints.txt"
    )
    configured.parent.mkdir(parents=True)
    configured.write_text("# test\n", encoding="utf-8")
    installed_module = (
        workspace
        / "install"
        / "wheelloong_auto_sdk"
        / "lib"
        / "python3.10"
        / "site-packages"
        / "wheelloong_auto_sdk"
        / "waypoints.py"
    )
    monkeypatch.delenv(WAYPOINT_FILE_ENV, raising=False)
    monkeypatch.setattr(
        waypoint_storage, "__file__", str(installed_module)
    )

    assert default_waypoint_path() == configured


def test_navigation_example_uses_sdk_runtime_waypoint_path(monkeypatch):
    """安装环境统一使用 SDK 解析出的可写用户数据路径。"""
    monkeypatch.delenv(WAYPOINT_FILE_ENV, raising=False)
    example = load_navigation_example()

    args = example.build_parser().parse_args(["1"])

    assert args.file == default_waypoint_path()


@pytest.mark.parametrize(
    ("waypoint_ids", "expected_action"),
    ((["1"], "navigate_to"), (["3", "1"], "navigate_through")),
)
def test_demo_calls_public_navigation_api_directly(
    tmp_path, monkeypatch, waypoint_ids, expected_action
):
    """把编号序列交给 SDK，由接口选择单点或多点 Action。"""
    path = tmp_path / "waypoints.txt"
    first = NavigationPose(1.0, 2.0, 0.3)
    third = NavigationPose(4.0, 5.0, -0.6)
    append_waypoint(first, waypoint_id=1, path=path)
    append_waypoint(third, waypoint_id=3, path=path)
    example = load_navigation_example()
    backend = MockBackend()
    robot = Robot.with_backend(backend)
    standalone = Mock(return_value=nullcontext(robot))
    monkeypatch.setattr(example.Robot, "standalone", standalone)
    monkeypatch.setattr(
        sys,
        "argv",
        ["waypoint_navigation_demo.py", *waypoint_ids, "--file", str(path)],
    )

    assert example.main() == 0
    assert [name for name, _ in backend.calls] == [
        "set_control_mode",
        "get_navigation_pose",
        "clear_local_costmap",
        "clear_global_costmap",
        expected_action,
        "set_control_mode",
    ]
    if expected_action == "navigate_to":
        assert backend.calls[4][1]["pose"] == first
    else:
        assert backend.calls[4][1]["poses"] == (third, first)
    assert backend.calls[0][1]["mode"] == int(ControlMode.AUTO)
    assert backend.calls[-1][1]["mode"] == int(ControlMode.IDLE)
    assert all(name != "set_enabled" for name, _ in backend.calls)


def test_demo_ctrl_c_cancels_the_active_navigation_goal(
    tmp_path, monkeypatch, capsys
):
    """Explicitly cancel and confirm the current goal before context exit."""
    path = tmp_path / "waypoints.txt"
    target = NavigationPose(1.0, 2.0, 0.3)
    append_waypoint(target, waypoint_id=1, path=path)
    example = load_navigation_example()
    backend = MockBackend()
    robot = Robot.with_backend(backend)
    accepted_handles = []
    original_navigate_to = robot.navigation.navigate_to

    monkeypatch.setattr(
        example.Robot, "standalone", Mock(return_value=nullcontext(robot))
    )
    monkeypatch.setattr(
        sys,
        "argv",
        ["waypoint_navigation_demo.py", "1", "--file", str(path)],
    )

    def capture_interrupting_handle(*args, **kwargs):
        """Return a real handle whose blocking wait simulates Ctrl+C."""
        handle = original_navigate_to(*args, **kwargs)
        accepted_handles.append(handle)
        handle.wait = Mock(side_effect=KeyboardInterrupt)
        return handle

    monkeypatch.setattr(
        robot.navigation, "navigate_to", capture_interrupting_handle
    )

    assert example.main() == 130
    assert len(accepted_handles) == 1
    assert accepted_handles[0].done
    assert accepted_handles[0].cancel().status == "CANCELED"
    assert [
        values["mode"]
        for name, values in backend.calls
        if name == "set_control_mode"
    ] == [int(ControlMode.AUTO), int(ControlMode.IDLE)]
    assert all(name != "set_enabled" for name, _ in backend.calls)
    output = capsys.readouterr()
    assert "active goal cancellation requested" in output.out


def test_demo_preserves_an_initial_auto_mode(tmp_path, monkeypatch):
    """Leave AUTO active when the Demo did not introduce that mode."""
    path = tmp_path / "waypoints.txt"
    append_waypoint(NavigationPose(1.0, 2.0, 0.3), path=path)
    example = load_navigation_example()
    backend = MockBackend()
    backend.state = replace(
        backend.state, control_mode=int(ControlMode.AUTO)
    )
    robot = Robot.with_backend(backend)
    monkeypatch.setattr(
        example.Robot, "standalone", Mock(return_value=nullcontext(robot))
    )
    monkeypatch.setattr(
        sys,
        "argv",
        ["waypoint_navigation_demo.py", "1", "--file", str(path)],
    )

    assert example.main() == 0
    assert [
        values["mode"]
        for name, values in backend.calls
        if name == "set_control_mode"
    ] == [int(ControlMode.AUTO)]
    assert all(name != "set_enabled" for name, _ in backend.calls)
