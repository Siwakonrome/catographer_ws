from setuptools import setup
import os
from glob import glob

package_name = 'yahboom_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['launch/Mapping_bring.launch.py' , 
                                   'launch/Catographer.launch.py',
                                   'launch/occupancy_grid.launch.py',
                                   'launch/rtabmap_2d_lidar_slam.launch.py']),

        ('share/' + package_name + '/params', ['params/car_2d.lua']),
        ('share/' + package_name + '/rvizs', glob('rvizs/*.rviz'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pi',
    maintainer_email='pi@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
