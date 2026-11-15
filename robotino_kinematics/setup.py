from setuptools import setup

package_name = 'robotino_kinematics'

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
    maintainer='kilian',
    maintainer_email='kilian@todo.todo',
    description='Paquete de cinemática y odometría para Robotino',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'local_to_wheel_velocities_node = robotino_kinematics.local_to_wheel_velocities_node:main',
            'odometry.py = robotino_kinematics.odometry:main'
        ],
    },
)