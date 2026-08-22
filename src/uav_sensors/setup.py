from setuptools import find_packages, setup
import os

package_name = 'uav_sensors'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), ['config/gazebo_bridge.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Alex',
    maintainer_email='alex@example.com',
    description='Мосты сенсоров: ros_gz_bridge + глубина в лазерный скан',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'depth_to_scan = uav_sensors.depth_to_scan:main',
            'odom_tf_broadcaster = uav_sensors.odom_tf_broadcaster:main',
        ],
    },
)
