from setuptools import setup
import os
from glob import glob

package_name = 'rc_car_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aa',
    maintainer_email='aa@todo.todo',
    description='Integrated donkeycar style control for ROS 2',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motor_driver = rc_car_control.motor_driver:main',
            'donkey_control = rc_car_control.donkey_control:main'
        ],
    },
)
