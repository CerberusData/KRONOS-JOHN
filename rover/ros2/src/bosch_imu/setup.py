from setuptools import setup

package_name = 'bosch_imu'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Camilo Alvis',
    maintainer_email='camiloalvis@kiwibot.com',
    description='IMU node for Medussa Project - It contains the ROS2 wrapper for the IMU Bosch BNO055',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'imu = bosch_imu.imu_node:main'
        ],
    },
)
