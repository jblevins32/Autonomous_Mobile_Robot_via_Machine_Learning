import os
from glob import glob
from setuptools import find_packages, setup

package_name = "ml_robotics_project"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["tests"]),
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
            "a_star_node = ml_robotics_project.AStarNode:main",
            "ppo_node = ml_robotics_project.PpoNode:main",
            "slam_node = ml_robotics_project.SlamNode:main",
            "yolo_node = ml_robotics_project.yolo_node_entry_point:main",
            "camera_presenter_node = ml_robotics_project.CameraPresenterNode:main",
        ],
    },
)
