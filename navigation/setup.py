import os
from glob import glob
from setuptools import find_packages, setup

package_name = "navigation"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*launch.[pxy][yma]*")),
        ),
        (
            os.path.join("share", package_name, "rviz"),
            glob(os.path.join("test", "*.rviz")),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="interfaces",
    maintainer_email="zanem@cox.net",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "global_planner = navigation.global_planner:main",
            "local_planner = navigation.local_planner:main",
            "global_tester = navigation.test_global:main",
            "motor_simulator = navigation.simulated_motor_endpoint:main",
            "visualize_graph = navigation.visualize_graph:main",
            "speed_node = navigation.speed_node:main",
            "obstacle_converter = navigation.zed_object_to_obstacle:main",
            "collision_detector = navigation.collision_detector:main",
            "display_global_path = navigation.display_global_path:main"
        ],
    },
)
