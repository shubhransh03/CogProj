from glob import glob
import os
from setuptools import find_packages, setup

package_name = "cogproj_bringup"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "rviz"), glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="veerobot",
    maintainer_email="veerobot@todo.todo",
    description="Bringup launch files and master configuration for CogProj.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "preflight_check = cogproj_bringup.preflight_check:main",
            "demo_status = cogproj_bringup.demo_status:main",
            "demo_map_node = cogproj_bringup.demo_map_node:main",
        ],
    },
)

