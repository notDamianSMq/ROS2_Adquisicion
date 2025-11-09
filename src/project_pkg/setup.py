from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'project_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # ADD THIS LINE to include all files from the 'launch' directory
        (os.path.join('share', package_name, 'launch'), glob('launch/*launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='radu',
    maintainer_email='radu@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'sensor_processor = project_pkg.sensor_processor_node:main',
            'obstacle_detector = project_pkg.obstacle_detector_node:main',
            'listener_points = project_pkg.cloud_node:main',
        ],
    },
)
