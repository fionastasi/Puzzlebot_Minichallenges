from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'puzzlebot_sim2'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', 
            glob(os.path.join('launch', '*.launch.py'))),
        ('share/' + package_name + '/urdf', 
            glob(os.path.join('urdf', '*.urdf'))),
        ('share/' + package_name + '/rviz', 
            glob(os.path.join('rviz', '*.rviz'))),
        ('share/' + package_name + '/config', 
            glob(os.path.join('config', '*.yaml'))),
        ('share/' + package_name + '/meshes', 
            glob(os.path.join('meshes', '*.stl'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Fiona',
    maintainer_email='fiona@example.com',
    description='Puzzlebot kinematic simulator with odometry and trajectory control',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'puzzlebot_sim2 = puzzlebot_sim2.puzzlebot_sim_node:main',
            'localization = puzzlebot_sim2.localization_node:main',
            'trajectory_generator = puzzlebot_sim2.trajectory_generator_node:main',
            'controller = puzzlebot_sim2.controller_node:main',
        ],
    },
)
