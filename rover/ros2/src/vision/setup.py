import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'vision'

setup(
    name=package_name,
    version='2.0.0',
    #packages=[package_name],
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml'])
    ],
    keywords="Ai, Computer Vision, OpenCV",
    url="put link here",
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='JohnBetaCode',
    maintainer_email='john@kiwibot.com',
    description='Packages with nodes for cameras launch/handlers, computer vision algorithms, and other vision utils',
    license='Apache License 2.0',
    #  tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'video_mapping = vision.video_mapping:main'
        ],
    },
)

