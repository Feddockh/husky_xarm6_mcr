from setuptools import setup
import os
from glob import glob

package_name = 'husky_xarm6_mcr_gz'

def get_all_files(directory):
    """Recursively get all files, preserving directory structure."""
    paths = []
    for (path, directories, filenames) in os.walk(directory):
        for filename in filenames:
            src = os.path.join(path, filename)
            dst = os.path.join('share', package_name, path)
            paths.append((dst, [src]))
    return paths

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        *get_all_files('models'),
        *get_all_files('worlds'),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Hayden Feddock',
    maintainer_email='hayden4feddock@gmail.com',
    description='Gazebo simulation tools for husky_xarm6_mcr',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gazebo_viewpoint_capture_node = husky_xarm6_mcr_gz.gazebo_viewpoint_capture_node:main',
            'generate_aruco_models = husky_xarm6_mcr_gz.generate_aruco_models:main',
        ],
    },
)
