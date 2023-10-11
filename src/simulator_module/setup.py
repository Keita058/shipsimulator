import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'simulator_module'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name,'launch'),glob('launch/*launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tabuchi',
    maintainer_email='tabuchi-keita-wm@ynu.jp',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'actuator_node = simulator_module.actuator_node:main',
            'controller_node = simulator_module.controller_node:main',
            'model_node = simulator_module.model_node:main',
            'sensor_node = simulator_module.sensor_node:main',
        ],
    },
)
