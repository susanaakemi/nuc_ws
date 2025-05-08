from setuptools import setup

package_name = 'xbox_control_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='TU_NOMBRE',
    maintainer_email='TU_CORREO@EXAMPLE.COM',
    description='Nodo para controlar el robot con Xbox',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'xbox_control_node = xbox_control_pkg.xbox_control_node:main',
            'double_ackermann_control_node = double_ackermann_control.double_ackermann_control_node:main',
        ],
    },
)

