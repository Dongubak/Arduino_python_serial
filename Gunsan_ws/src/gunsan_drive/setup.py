import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'gunsan_drive'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'),
         glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gardentech',
    maintainer_email='x001125@gmail.com',
    description='Gunsan autonomous driving: lane detection with Logitech C920e cameras',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lane_detector        = gunsan_drive.lane_detector_node:main',
            'steering_controller  = gunsan_drive.steering_controller_node:main',
        ],
    },
)
