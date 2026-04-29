from setuptools import find_packages, setup

package_name = 'puzzlebot_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/puzzlebot_sim/meshes', ['meshes/Puzzlebot_Jetson_Lidar_Edition_Base.stl', 
                                        'meshes/Puzzlebot_Caster_Wheel.stl', 'meshes/Puzzlebot_Wheel.stl']),
        ('share/puzzlebot_sim/urdf', ['urdf/puzzlebot.urdf']),
        ('share/puzzlebot_sim/launch', ['launch/display.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='danieldrg',
    maintainer_email='dreggam@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_markers = puzzlebot_sim.robot_markers:main',
            'URDF_tfs= puzzlebot_sim.URDF_tfs:main',
            'kinematic_model = puzzlebot_sim.kinematic_model:main',
            'localisation = puzzlebot_sim.localisation:main',
            'control = puzzlebot_sim.control:main',
        ],
    },
)
