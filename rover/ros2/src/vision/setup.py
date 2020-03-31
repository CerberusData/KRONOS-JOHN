import os
from glob import glob
from setuptools import setup

package_name = 'vision'

setup(
    name=package_name,
    version='2.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dev-john',
    maintainer_email='john@kiwibot.com',
    description='Nodes for cameras launch, computer vision algorithms, and other vision utils',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'video_mapping = vision.video_mapping:main'
        ],
    },
)
