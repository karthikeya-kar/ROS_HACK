from setuptools import setup
import os
from glob import glob

package_name = "ros_hack"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "worlds"), glob("worlds/*")),
        (os.path.join("share", package_name, "drone_urdf"), glob("drone_urdf/*")),
    ],
    install_requires=["setuptools"],
    entry_points={
        "console_scripts": [
            "autonomous = src.autonomous:main",
        ],
    },
)