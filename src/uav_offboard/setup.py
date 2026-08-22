from setuptools import find_packages, setup
import os

package_name = 'uav_offboard'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'),
         ['config/offboard.yaml', 'config/nav2_commander.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Alex',
    maintainer_email='alex@example.com',
    description='Оффборд-управление PX4: trajectory_setpoint, MPC, Безье',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'offboard_node = uav_offboard.offboard_node:main',
            'nav2_commander_node = uav_offboard.nav2_commander_node:main',
        ],
    },
)
