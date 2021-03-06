from setuptools import setup
import os
from glob import glob

package_name = 'noisy_sensors'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='andrepadilha',
    maintainer_email='andrejpadilha@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'noisyOdom = noisy_sensors.noisy_odom:main',
        	'noisy_pose_in_x_turtlesim = noisy_sensors.noisy_pose_in_x_turtlesim:main',
        	'noisy_pose_in_xy_turtlesim = noisy_sensors.noisy_pose_in_xy_turtlesim:main'
        ],
    },
)
