from setuptools import setup

package_name = 'interplanetar_rover'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Team Interplanetar BUET',
    maintainer_email='interplanetar@buet.ac.bd',
    description='ROS2 Humble rover control stack',
    license='MIT',
    entry_points={
        'console_scripts': [
            'gui_node          = interplanetar_rover.gui_node:main',
            'wheel_bridge_node = interplanetar_rover.wheel_bridge_node:main',
            'arm_bridge_node   = interplanetar_rover.arm_bridge_node:main',
        ],
    },
)
