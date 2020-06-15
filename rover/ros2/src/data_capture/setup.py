import os
from glob import glob
from setuptools import setup, find_packages

package_name = "data_capture"

setup(
    name=package_name,
    version="2.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    keywords="Ai, Computer Vision, OpenCV",
    url="https://www.kiwibot.com/",
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="dev-john",
    maintainer_email="john.betancourt93@gmail.com",
    description="package with everything for robot's data capture",
    license="do whatever you want",
    # tests_require=["pytest"],
    entry_points={
        "console_scripts": ["data_capture = data_capture.node_data_capture:main",],
    },
)
