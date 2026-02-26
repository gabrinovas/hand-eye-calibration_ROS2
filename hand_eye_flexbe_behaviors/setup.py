#!/usr/bin/env python
import os
from setuptools import setup

package_name = 'hand_eye_flexbe_behaviors'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'manifest'), 
         [os.path.join('manifest', 'camera_calibration.xml'),
          os.path.join('manifest', 'manual_hand_eye_calibration.xml'),
          os.path.join('manifest', 'verify_calibraion.xml'),
          os.path.join('manifest', 'capture_and_calibrate.xml')]),
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
            'manual_hand_eye_calibration_sm = hand_eye_flexbe_behaviors.manual_hand_eye_calibration_sm',
            'verify_calibraion_sm = hand_eye_flexbe_behaviors.verify_calibraion_sm',
            'capture_and_calibrate_sm = hand_eye_flexbe_behaviors.capture_and_calibrate_sm',
        ],
    },
)
