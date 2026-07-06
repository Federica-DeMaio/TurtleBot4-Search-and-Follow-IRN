import os
from glob import glob

from setuptools import find_packages, setup

package_name = "irn_g01"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*.py")),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Federico",
    maintainer_email="feder@users.noreply.github.com",
    description="ROS 2 nodes for ArUco marker detection and predictive following.",
    license="Apache-2.0",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "aruco_reader = irn_g01.aruco_reader:main",
            "compute_pose = irn_g01.compute_pose:main",
            "BT_FSM_follow = irn_g01.BT_FSM_follow:main",
        ],
    },
)
