from setuptools import find_packages, setup
import os
from glob import glob

package_name = "mobile_robot_maxime"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    # Everything installed in ./install
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "config"),
            glob(os.path.join("config", "*.[yaml]*")),
        ),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*launch.[pxy][yma]*")),
        ),
        (
            os.path.join("share", package_name, "worlds"),
            glob(os.path.join("worlds", "*.[sdf]*")),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="maxime.leriche",
    maintainer_email="maxime.leriche@tparlem03.cpe.lan",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "velocity_pub = mobile_robot_maxime.publisher_velocity:main",
            "secured_vel = mobile_robot_maxime.secured_cmd_vel_pub:main",
            "send_cmd_vel = mobile_robot_maxime.client_cmd_vel:main",
            "breakdance = mobile_robot_maxime.test_cmd_ee:main",
            "keyboard = mobile_robot_maxime.keyboard:main",
        ],
    },
)
