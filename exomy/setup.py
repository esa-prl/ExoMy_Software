import os
from glob import glob
from setuptools import setup

package_name = 'exomy'

setup(
    name=package_name,
    version='2.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
        (os.path.join('share', package_name), glob('config/*.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Maximilian Ehrhardt',
    maintainer_email='max.ehrhardt@hotmail.de',
    description='Core functionalities for the ExoMy rover',
    license='GPLv3',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motor_node = exomy.motor_node:main',
            'gamepad_parser_node = exomy.gamepad_parser_node:main',
            'robot_node = exomy.robot_node:main',
        ],
    },
)
