from setuptools import find_packages, setup

package_name = 'odometry'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools','numpy','pyserial','tf-transformations'],
    zip_safe=True,
    maintainer='robotec',
    maintainer_email='robotec@todo.todo',
    description='Paquete que contiene los cálculos de odometría y sus funciones auxiliares',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
		'odometry_node = odometry.odometry_node:main'
        ],
    },
)
