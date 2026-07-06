import os
from glob import glob

from setuptools import find_packages, setup

package_name = "tb4_project"

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
            glob(os.path.join("launch", "*.launch.py")),
        ),
        (
            os.path.join("share", package_name, "config"),
            glob(os.path.join("config", "*.yaml")),
        ),
        (
            os.path.join("share", package_name, "behavior_trees"),
            glob(os.path.join("behavior_trees", "*.xml")),
        ),
        (
            os.path.join("share", package_name, "rviz"),
            glob(os.path.join("rviz", "*.rviz")),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Federico",
    maintainer_email="feder@users.noreply.github.com",
    description="ROS 2 launch files, Nav2 configuration and waypoint mission tools.",
    license="Apache-2.0",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "search_mission = tb4_project.prova_waypoint_mission:main",
            "compute_cost_matrix = tb4_project.planner_cost_matrix:main",
            "recompute_route_costs = tb4_project.recompute_route_costs:main",
        ],
    },
)
