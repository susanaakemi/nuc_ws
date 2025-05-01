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
        'numpy<2'                # ← here!
    ],
    zip_safe=True,
    maintainer='robotec',
    maintainer_email='a01640939@tec.mx',
    description='Detección de rocas con YOLOv8 en ROS 2 Humble',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'framepublisher = camera_pkg.framepublisher:main',
            'pixel_to_point3d = camera_pkg.pixel_to_point3d:main',
        ],
    },
)
