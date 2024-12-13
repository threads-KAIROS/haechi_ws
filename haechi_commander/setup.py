from setuptools import find_packages, setup

package_name = 'haechi_commander'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pytest'],  # pytest를 install_requires에 추가
    zip_safe=True,
    maintainer='jhchoman',
    maintainer_email='jhchoman0823@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'vehicle_navigation = haechi_commander.vehicle_navigation:main',
        ],
    },
)
