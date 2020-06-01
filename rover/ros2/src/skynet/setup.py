import os
from glob import glob
from setuptools import setup, find_packages

package_name = "skynet"

setup(
    name=package_name,
    version="0.0.1",
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
    description="package with everything of machine learning nodes",
    license="do whatever you want",
    # tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "object_detector = skynet.node_skynet_object_detector:main",
        ],
    },
)
