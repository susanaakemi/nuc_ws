from setuptools import find_packages, setup

package_name = 'odometry_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'numpy',
        'tf-transformations',
        'pyserial'],
    zip_safe=True,
    maintainer='robotec',
    maintainer_email='a01640939@tec.mx',
    description='Odometry package with EKF, IMU listener, and waypoint control',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odometry_node = odometry_pkg.odometry_node:main',
            'imu_vesc_listener = odometry_pkg.utils:main',
            'rpm_test_publisher = odometry_pkg.rpm_test_publisher:main',
            'rpmtrue_publisher = odometry_pkg.rpmtrue_publisher:main',
        ],
    },
)
