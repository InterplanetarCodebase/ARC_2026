from setuptools import setup
import os
from glob import glob

package_name = 'interplanetar_rover'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    package_data={package_name: ['image.png']},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'gui_node          = interplanetar_rover.gui_node:main',
            'wheel_bridge_node = interplanetar_rover.wheel_bridge_node:main',
            'arm_bridge_node   = interplanetar_rover.arm_bridge_node:main',
            'camera_receiver_node    = interplanetar_rover.camera_receiver_node:main',
            'camera_transmitter_node = interplanetar_rover.camera_transmitter_node:main',
        ],
    },
)