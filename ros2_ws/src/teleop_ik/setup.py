import os
from glob import glob

from setuptools import find_packages, setup

package_name = "teleop_ik"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="OJII3",
    maintainer_email="ojii3dev@gmail.com",
    description="VR teleop IK node for SO-101 arm using Pinocchio",
    license="MIT",
    entry_points={
        "console_scripts": [
            "ik_node = teleop_ik.ik_node:main",
        ],
    },
)
