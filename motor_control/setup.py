import os
from glob import glob
from setuptools import find_packages, setup

package_name = "motor_control"

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
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="student",
    maintainer_email="student@todo.todo",
    description="Motor control package that interacts with the arduino board.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "motor_endpoint = motor_control.motor_endpoint:main",
            "motor_endpoint_test = motor_control.motor_endpoint_test:main",
        ],
    },
)
