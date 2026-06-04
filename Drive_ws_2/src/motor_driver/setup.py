from setuptools import find_packages, setup

package_name = 'motor_driver'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='gardentech',
    maintainer_email='x001125@gmail.com',
    description='ROS2 serial motor driver using binary protocol for Arduino L298N controller',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motor_driver_node = motor_driver.motor_driver_node:main',
        ],
    },
)
