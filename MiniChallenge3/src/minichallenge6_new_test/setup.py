from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'minichallenge6_new_test'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*.stl')),
        ('share/minichallenge6_new_test/urdf', ['urdf/puzzlebot.urdf']),
        ('share/minichallenge6_new_test/launch', ['launch/puzzlebot_gazebo.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aaraizae',
    maintainer_email='aaraizae@hotmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'test_node = minichallenge6_new_test.test_node:main',
            "localisation = minichallenge6_new_test.localisation:main",
            "robot_markers = minichallenge6_new_test.robot_markers:main",
            "URDF_tfs = minichallenge6_new_test.URDF_tfs:main",
        ],
    },
)
