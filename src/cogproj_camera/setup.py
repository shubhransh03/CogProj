from setuptools import find_packages, setup

package_name = "cogproj_camera"

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
    description="Camera input component for CogProj ingesting Beetle Bot camera frames.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "camera_input_node = cogproj_camera.camera_input_node:main",
        ],
    },
)

