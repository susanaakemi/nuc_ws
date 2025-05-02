from setuptools import setup

package_name = 'odometry'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='Odometry package with EKF and control nodes',
    license='Apache License 2.0',
    tests_require=['pytest'],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/msg', [
            'msg/WheelInfo.msg',
            'msg/WheelRPMs.msg',
            'msg/DirectionAngles.msg'
        ]),
    ],
    entry_points={
        'console_scripts': [
            'odometry_node = odometry.odometry_node:main',
        ],
    },
)

