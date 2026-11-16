#!/usr/bin/env python
import os
from setuptools import setup
from glob import glob

package_name = 'hand_eye_flexbe_behaviors'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        # Archivo de recurso
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        
        # Package.xml
        ('share/' + package_name, ['package.xml']),
        
        # MANIFIESTOS
        (os.path.join('lib', package_name, 'manifest'),
         glob('manifest/*.xml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Gabriel Novas',
    maintainer_email='gabriel.novas@aimen.es',
    description='Behaviors for hand-eye calibration with MoveIt and FlexBE',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_calibration_sm = hand_eye_flexbe_behaviors.camera_calibration_sm',
            'capture_and_calibrate_sm = hand_eye_flexbe_behaviors.capture_and_calibrate_sm',
            'test_detection_only_sm = hand_eye_flexbe_behaviors.test_detection_only_sm',
        ],
    },
)
