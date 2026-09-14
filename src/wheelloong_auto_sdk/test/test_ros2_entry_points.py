"""Validate every readable example is exposed through ros2 run."""

from importlib.metadata import distribution


EXPECTED_EXECUTABLES = {
    "auto_oscillation_demo",
    "auto_session_demo",
    "mobile_manipulation_demo",
    "plot_oscillation_targets",
    "separation_demo",
    "simulate_oscillation_targets",
    "voice_demo",
    "waypoint_navigation_demo",
    "waypoint_recorder",
}


def test_all_examples_have_loadable_console_entry_points():
    """Load every installed console target without executing its main()."""
    installed = {
        entry.name: entry
        for entry in distribution("wheelloong_auto_sdk").entry_points
        if entry.group == "console_scripts"
    }

    assert EXPECTED_EXECUTABLES <= set(installed)
    for name in EXPECTED_EXECUTABLES:
        assert callable(installed[name].load())
