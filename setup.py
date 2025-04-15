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
    ],
    install_requires=["setuptools"],
    zip_safe=True,
<<<<<<< HEAD:src/eeg_publisher/setup.py
    maintainer="root",
    maintainer_email="nik.k.nikaznan@outlook.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
=======
    maintainer='Ahmed Al-Hindawi',
    maintainer_email='a.al-hindawi@ucl.ac.uk',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
>>>>>>> d1ae316109d0db7b78d7666180a9a63329a0e774:setup.py
    entry_points={
        "console_scripts": [
            "mock_publisher = eeg_publisher.mock_publisher:main",
        ],
    },
)
