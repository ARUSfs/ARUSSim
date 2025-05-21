from setuptools import find_packages
from setuptools import setup

setup(
    name='arussim_msgs',
    version='0.0.0',
    packages=find_packages(
        include=('arussim_msgs', 'arussim_msgs.*')),
)
