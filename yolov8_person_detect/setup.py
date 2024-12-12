from setuptools import find_packages, setup

package_name = 'yolov8_person_detect'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/yolo_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jhchoman',
    maintainer_email='jhchoman0823@gmail.com',
    description='YOLOv8-based person detection ROS 2 package',
    license='MIT',
    entry_points={
        'console_scripts': [
            'yolo_node = yolov8_person_detect.yolo_node:main'
        ],
    },
)
