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
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Maximilian Ehrhardt',
    maintainer_email='max.ehrhardt@hotmail.de',
    description='Control functionalities for the ExoMy rover',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motors = py_pubsub.publish_member_function:main',
            'joystick = py_pubsub.publish_member_function:main', 
        ],
    },
)
