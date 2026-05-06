import os
from glob import glob

from setuptools import setup


package_name = 'vrx_vln_bringup'


setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='THUlzq',
    maintainer_email='luzq25@mails.tsinghua.edu.cn',
    description='Single-USV empty-scene bringup package for the VRX_VLN workspace.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'sensor_marker_publisher = vrx_vln_bringup.sensor_marker_publisher:main',
        ],
    },
)
