"""Setuptools and ament_python metadata for wheelloong_auto_sdk."""

from setuptools import find_packages, setup


PACKAGE_NAME = "wheelloong_auto_sdk"
EXAMPLES_PACKAGE = "wheelloong_auto_sdk_examples"

EXAMPLE_ENTRY_POINTS = [
    "auto_oscillation_demo = "
    f"{EXAMPLES_PACKAGE}.auto_oscillation_demo:main",
    "auto_session_demo = "
    f"{EXAMPLES_PACKAGE}.auto_session_demo:main",
    "existing_node_demo = "
    f"{EXAMPLES_PACKAGE}.existing_node_demo:main",
    "plot_oscillation_targets = "
    f"{EXAMPLES_PACKAGE}.plot_oscillation_targets:main",
    "simulate_oscillation_targets = "
    f"{EXAMPLES_PACKAGE}.simulate_oscillation_targets:main",
    "standalone_demo = "
    f"{EXAMPLES_PACKAGE}.standalone_demo:main",
    "waypoint_navigation_demo = "
    f"{EXAMPLES_PACKAGE}.waypoint_navigation_demo:main",
    "waypoint_recorder = "
    f"{EXAMPLES_PACKAGE}.waypoint_recorder:main",
]


setup(
    name=PACKAGE_NAME,
    version="0.1.0",
    packages=(
        find_packages(
            exclude=("test", "test.*", "examples", "examples.*")
        )
        + [EXAMPLES_PACKAGE]
    ),
    package_dir={EXAMPLES_PACKAGE: "examples"},
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + PACKAGE_NAME],
        ),
        ("share/" + PACKAGE_NAME, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="zeropanga",
    maintainer_email="zeropanga@buaa.com",
    description="Stable Python task SDK for Wheelloong/Shiloong AUTO mode.",
    license="Proprietary",
    tests_require=["pytest"],
    entry_points={"console_scripts": EXAMPLE_ENTRY_POINTS},
)
