from setuptools import find_packages, setup
import os
from glob import glob

package_name = "eeg_publisher"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob("launch/*.launch.*") + glob("launch/*.py"),
        ),
        (
            os.path.join("share", package_name, "config"),
            glob("config/*.yaml"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Ahmed Al-Hindawi",
    maintainer_email="a.al-hindawi@ucl.ac.uk",
    description="EEG Publisher for ROS2",
    license="Apache License 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "mock_publisher = eeg_publisher.mock_publisher:main",
        ],
    },
)
