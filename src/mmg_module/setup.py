from setuptools import setup

package_name = 'mmg_module'

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
    maintainer='tabuchi',
    maintainer_email='tabuchi@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
          "model_node = mmg_module.model_node:main",
          "controller_node = mmg_module.controller_node:main",
          "controller_key_node = mmg_module.controller_key_node:main",
        ],
    },
)
