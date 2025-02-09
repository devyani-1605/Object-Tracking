from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'object_tracking'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, glob('launch/*.launch.[pxy][yma]*')),  # ✅ Fixed path
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='devyani',
    maintainer_email='devyania161@gmail.com',
    description='Object detection and tracking with YOLOv5 and RealSense in ROS2',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'Publisher = object_tracking.Publisher:main',
            'Subscriber = object_tracking.Subscriber:main',
        ],
    },
)

