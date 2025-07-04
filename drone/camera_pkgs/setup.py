from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'camera_pkgs'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
    ],
    
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rpi5',
    maintainer_email='rpi5@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'trigger_node = camera_pkgs.trigger_node:main',
            'publish_lidar = camera_pkgs.publish_lidar:main',
            'publish_magnetic_declination = camera_pkgs.publish_magnetic_declination:main',            
        ],
    },
)
