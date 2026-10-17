from glob import glob

from setuptools import find_packages, setup

package_name = 'gripper_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/meshes', glob('meshes/*')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Till Laube',
    maintainer_email='tilllaube@gmail.com',
    description='Computes and commands gripper movements on top of trunk_control',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'gripper_control_node = gripper_control.gripper_control_node:main',
        ],
    },
)
