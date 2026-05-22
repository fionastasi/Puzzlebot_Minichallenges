from setuptools import find_packages, setup

package_name = 'minichallenge5'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/minichallenge5/meshes', ['meshes/Puzzlebot_Jetson_Lidar_Edition_Base.stl', 
                                        'meshes/Puzzlebot_Caster_Wheel.stl', 'meshes/Puzzlebot_Wheel.stl']),
        ('share/minichallenge5/urdf', ['urdf/puzzlebot.urdf']),
        ('share/minichallenge5/launch', ['launch/display.launch.py']),
        ('share/minichallenge5/launch', ['launch/combined.launch.py']),
        ('share/minichallenge5/launch', ['launch/minichallenge6.launch.py']),
        ('share/minichallenge5/launch', ['launch/multi_robot.launch.py']),
        ('share/minichallenge5/launch', ['launch/covariance_test.launch.py']),
        ('share/minichallenge5/launch', ['launch/gazebo.launch.py']),
        ('share/minichallenge5/launch', ['launch/robot_spawn.launch.py']),
        ('share/minichallenge5/rviz', ['rviz/setup.rviz']),
        ('share/minichallenge5/worlds', ['worlds/bug0_test.world']),
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
            'robot_markers = minichallenge5.robot_markers:main',
            'URDF_tfs= minichallenge5.URDF_tfs:main',
            'kinematic_model = minichallenge5.kinematic_model:main',
            'localisation = minichallenge5.localisation:main',
            'control = minichallenge5.control:main',
            'bug0 = minichallenge5.bug0:main',
            'bug1 = minichallenge5.bug1:main',
        ],
    },
)
