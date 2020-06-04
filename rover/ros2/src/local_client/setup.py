import os
from glob import glob
from setuptools import setup, find_packages

package_name = "local_client"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    keywords="local_client, socketio",
    url="https://www.kiwibot.com/",
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="dev-john",
    maintainer_email="john.betancourt93@gmail.com",
    description="package for local client control",
    license="do whatever you want",
    # tests_require=["pytest"],
    entry_points={
        "console_scripts": ["local_client = local_client.node_local_client:main",],
    },
)
