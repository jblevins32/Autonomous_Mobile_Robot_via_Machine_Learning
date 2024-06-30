import os
from glob import glob
from setuptools import find_packages, setup

package_name = "ml_robotics_project"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        # Permit launch files
        (os.path.join("share", package_name), glob("launch/*launch.[pxy][yma]*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Jacob Blevins",
    maintainer_email="jacob.blevins@gatech.edu",
    description="For CS7641 final group project",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            # Put entry points here
            # "controller = ftm_project.controller_node:main",
            # "multilateration = ftm_project.multilateration_node:main",
            # "ftm_publisher = ftm_project.ftm_publisher_node:main",
            # 'ftm_server = ftm_project.ftm_server_node:main',
            # 'ftm_remote_server = ftm_project.ftm_remote_server_node:main',
            # 'ftm_teleop = ftm_project.ftm_teleop:main'
        ],
    },
)
