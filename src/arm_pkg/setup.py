from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'arm_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robotec',
    maintainer_email='a01640939@tec.mx',
    description='Pick-and-place arm controller with serial communication',
    license='Apache-2.0',  # ← update with your real license
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'clicked_point_subscriber = arm_pkg.clicked_point_subscriber:main',
        ],
    },
)
