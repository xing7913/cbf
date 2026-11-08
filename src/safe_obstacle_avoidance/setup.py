from glob import glob
import os
from setuptools import find_packages, setup

WORLD_FILES = glob("worlds/*.sdf")

package_name = "safe_obstacle_avoidance"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "worlds"), WORLD_FILES),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml") + glob("config/*.rviz")),
    ],
    install_requires=["setuptools", "numpy"],
    zip_safe=True,
    maintainer="safe-obstacle-avoidance-ros2",
    maintainer_email="maintainer@example.com",
    description="ROS 2 depth image ellipsoid safe sets and CBF-QP obstacle avoidance.",
    license="NOASSERTION",
    entry_points={
        "console_scripts": [
            "safe_set_generator = safe_obstacle_avoidance.safe_set_generator:main",
            "cbf_controller = safe_obstacle_avoidance.cbf_controller:main",
            "reference_cmd_publisher = safe_obstacle_avoidance.reference_cmd_publisher:main",
            "cbf_visualization = safe_obstacle_avoidance.visualization_markers:main",
        ],
    },
)
