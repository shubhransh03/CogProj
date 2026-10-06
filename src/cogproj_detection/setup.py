from setuptools import find_packages, setup

package_name = "cogproj_detection"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="veerobot",
    maintainer_email="veerobot@todo.todo",
    description="ROS 2 detection node for CogProj using pluggable detector adapters.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "detection_node = cogproj_detection.detection_node:main",
        ],
    },
)

