import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'tribo_nav_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*.launch.py"))
            + glob(os.path.join("launch", "*.xml")),
        ),
        (
            os.path.join("share", package_name, "config"),
            glob(os.path.join("config", "*.yaml")),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hj',
    maintainer_email='xxbb96@gmail.com',
    description='Nav2 프로그래밍 제어 클라이언트 (목표 전송/대기/취소)',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'goal_sender = tribo_nav_control.goal_sender:main',
            'waypoint_follower = tribo_nav_control.waypoint_follower:main',
        ],
    },
)
