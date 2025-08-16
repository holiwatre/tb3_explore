# setup.py
from setuptools import setup

package_name = 'tb3_sim_bundle'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/worlds', ['worlds/maze_3x3.world']),
        ('share/' + package_name + '/launch', ['launch/gazebo_tb3.launch.py', 'launch/spawn_tb3.launch.py', 'launch/sim_explore.launch.py']),
        ('share/' + package_name + '/maps', ['maps/maze_3x3.yaml']),
        ('share/' + package_name + '/tools', ['tools/make_test_maps.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='Gazebo sim & maps bundle for TB3',
    license='MIT',
)
