from setuptools import find_packages, setup
import os

package_name = 'uav_perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), ['config/perception.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Alex',
    maintainer_email='alex@example.com',
    description='Компьютерное зрение: OpenCV/YOLO, детектор H-маркера',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'marker_detector = uav_perception.marker_detector:main',
            'human_detector = uav_perception.human_detector:main',
            'target_localizer = uav_perception.target_localizer:main',
        ],
    },
)
