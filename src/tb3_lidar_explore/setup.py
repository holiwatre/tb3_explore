from setuptools import setup

package_name = 'tb3_lidar_explore'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/bringup.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='Frontier-based autonomous exploration for TurtleBot3 using /map and /scan.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'frontier_explorer = tb3_lidar_explore.frontier_explorer:main',
        ],
    },
)
