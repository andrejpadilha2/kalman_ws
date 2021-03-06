from setuptools import setup
import os
from glob import glob

package_name = 'kalman_1d'

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
        	'kf_1d_turtlesim_v1 = kalman_1d.kf_1d_turtlesim_v1:main',
        	'kf_1d_turtlesim_v0 = kalman_1d.kf_1d_turtlesim_v0:main',
        ],
    },
)
