from setuptools import setup
import os
from glob import glob

package_name = 'kalman_filter_nd'

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
    maintainer='andrejpadilha',
    maintainer_email='andrejpadilha@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'kf_nd_turtlesim_in_x_v0 = kalman_filter_nd.kf_nd_turtlesim_in_x_v0:main',
        	'kf_nd_turtlesim_in_xy_v0 = kalman_filter_nd.kf_nd_turtlesim_in_xy_v0:main',
        	'kf_sensor_fusion_turtlesim_in_x_v0 = kalman_filter_nd.kf_sensor_fusion_turtlesim_in_x_v0:main',
        	'kf_sensor_fusion_turtlesim_in_xy_v0 = kalman_filter_nd.kf_sensor_fusion_turtlesim_in_xy_v0:main',
        	'kf_sensor_fusion_turtlesim_in_x_control_v0 = kalman_filter_nd.kf_sensor_fusion_turtlesim_in_x_control_v0:main',
        	'kf_sensor_fusion_turtlesim_in_xy_control_v0 = kalman_filter_nd.kf_sensor_fusion_turtlesim_in_xy_control_v0:main',
        	
        ],
    },
)
