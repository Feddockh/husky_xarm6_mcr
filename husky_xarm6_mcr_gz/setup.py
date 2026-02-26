from setuptools import find_packages, setup

package_name = 'husky_xarm6_mcr_gz'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hayden',
    maintainer_email='hayden4feddock@gmail.com',
    description='Gazebo simulation tools for husky_xarm6_mcr',
    license='MIT',
    entry_points={
        'console_scripts': [
            'gazebo_viewpoint_capture_node = husky_xarm6_mcr_gz.gazebo_viewpoint_capture_node:main',
        ],
    },
)
