from setuptools import setup
import os
from glob import glob

package_name = 'obstackle_avoidance'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')), # Launch files
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')), # RViz config files
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')), # World files
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    description='',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'emergency_braking = obstackle_avoidance.emergency_braking:main', # Python script (virtual)
            'wall_following = obstackle_avoidance.wall_following:main', # Python script (virtual)
            'obstacle_avoidance = obstackle_avoidance.obstacle_avoidance:main', # Python script (virtual)
            'collision_avoidance = obstackle_avoidance.collision_avoidance:main', # Python script (reality)
        ],
    },
)
