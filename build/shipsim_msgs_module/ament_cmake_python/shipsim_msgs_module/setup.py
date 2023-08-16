from setuptools import find_packages
from setuptools import setup

setup(
    name='shipsim_msgs_module',
    version='0.0.0',
    packages=find_packages(
        include=('shipsim_msgs_module', 'shipsim_msgs_module.*')),
)
