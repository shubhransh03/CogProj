from setuptools import find_packages, setup

package_name = 'cogproj_visualization'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='veerobot',
    maintainer_email='veerobot@todo.todo',
    description='Real-time perception HUD and laptop viewer for CogProj ore detection and tracking',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'visualizer_node = cogproj_visualization.visualizer_node:main',
            'cogproj_viewer = cogproj_visualization.viewer:main',
        ],
    },
)

