from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'tribo_odom'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/odom.launch.py']),
        ('share/' + package_name + '/config',
           glob(os.path.join('config', '*.yaml'))),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hj',
    maintainer_email='xxbb96@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'odom_publisher = tribo_odom.odom_publisher:main',
            'odom_source = tribo_odom.odom_source:main',
            'encoder_calib_test = tribo_odom.encoder_calib_test:main',
        ],
    },
)
