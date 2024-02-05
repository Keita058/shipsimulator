from setuptools import find_packages, setup

package_name = 'guidance_module'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'LOSguidance_node = guidance_module.LOSguidance_node:main',
            'PPguidance_node = guidance_module.PPguidance_node:main',
        ],
    },
)
