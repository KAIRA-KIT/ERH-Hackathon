from glob import glob

from setuptools import find_packages, setup

package_name = 'trunk_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Till Laube',
    maintainer_email='tilllaube@gmail.com',
    description='Basic control node for six daisy-chained Waveshare ST3215-HS servos',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'trunk_control_node = trunk_control.trunk_control_node:main',
        ],
    },
)
