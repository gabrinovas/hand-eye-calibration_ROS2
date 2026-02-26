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
            'charuco_camera_calibration = hand_eye_flexbe_states.charuco_camera_calibration',
            'compute_calib = hand_eye_flexbe_states.compute_calib',
            'find_charuco = hand_eye_flexbe_states.find_charuco',
            'move_robot_manually = hand_eye_flexbe_states.move_robot_manually',
            'take_picture = hand_eye_flexbe_states.take_picture',
            'take_pose_and_picture = hand_eye_flexbe_states.take_pose_and_picture',
            'offline_find_charuco = hand_eye_flexbe_states.offline_find_charuco',
            'launch_moveit = hand_eye_flexbe_states.launch_moveit',
        ],
    },
)
