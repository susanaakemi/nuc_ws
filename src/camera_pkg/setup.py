from setuptools import find_packages, setup

package_name = 'camera_pkg'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'opencv-python',
        'ultralytics',
        'numpy'
    ],
    zip_safe=True,
    maintainer='robotec',
    maintainer_email='a01640939@tec.mx',
    description='Detección de rocas con YOLOv8 en ROS 2 Humble',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rock_detector = camera_pkg.rock_detector_node:main',
        ],
    },
)