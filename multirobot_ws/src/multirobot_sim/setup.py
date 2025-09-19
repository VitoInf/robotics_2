from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'multirobot_sim'

def get_data_files(directory):
    """Get all files in directory while preserving structure."""
    data_files = []
    for root, _, files in os.walk(directory):
        if files:
            target = os.path.join('share', package_name, root)
            sources = [os.path.join(root, f) for f in files]
            data_files.append((target, sources))
    return data_files

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        *get_data_files('models'), 
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='test_user',
    maintainer_email='test_user@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'leader_controller = multirobot_sim.leader_controller:main',
        'trajectory_generator = multirobot_sim.trajectory_generator:main',
        'follower_controller = multirobot_sim.follower_controller:main',
        ],
    },
)