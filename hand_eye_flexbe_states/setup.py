#!/usr/bin/env python
from setuptools import setup
from setuptools import find_packages

package_name = 'hand_eye_flexbe_states'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Gabriel Novas',
    maintainer_email='gabriel.novas@aimen.es',
    description='FlexBE states for hand-eye calibration with MoveIt',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
