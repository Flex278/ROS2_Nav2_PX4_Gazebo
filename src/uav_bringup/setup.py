from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'uav_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Alex',
    maintainer_email='alex@example.com',
    description='Launch-файлы верхнего уровня для полной симуляции UAV',
    license='MIT',
    tests_require=['pytest'],
    entry_points={'console_scripts': []},
)
