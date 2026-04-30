#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, Logger
from hand_eye_flexbe_states.offline_find_charuco import OfflineFindCharucoState
from hand_eye_flexbe_states.compute_calib import ComputeCalibState
import os
# [MANUAL_IMPORT]
# [/MANUAL_IMPORT]


'''
Created on Thu Mar 04 2026
@author: Gabriel Novas
'''
class TestDetectionOnlySM(Behavior):
    """
    Test behavior ONLY for detection and calibration.
    
    Requires that the following already exist:
    - Images in /calibrations/extrinsic_calibration/pictures/
    - Poses in /calibrations/extrinsic_calibration/robot_poses/
    - Camera intrinsic calibration
    
    RESULTS: 
    - /home/drims/calibrations/camera_extrinsics.yaml (main)
    - /home/drims/calibrations/extrinsic_calibration/charuco_table_poses/ (detections)
    """
    
    def __init__(self, node):
        super(TestDetectionOnlySM, self).__init__()
        self.name = 'Test Detection Only'
        self.node = node

        # Behavior parameters
        self.add_parameter('eye_in_hand', False)
        self.add_parameter('calibration_file_name', 'camera_extrinsics.yaml')
        
        # PATHS - Correctly defined
        base_calib_path = '/home/drims/calibrations'
        self.add_parameter('pictures_folder', f'{base_calib_path}/extrinsic_calibration/pictures')
        self.add_parameter('robot_poses_folder', f'{base_calib_path}/extrinsic_calibration/robot_poses')
        self.add_parameter('charuco_output_folder', f'{base_calib_path}/extrinsic_calibration/charuco_table_poses')
        self.add_parameter('output_folder', base_calib_path)

        # Initialize states
        OfflineFindCharucoState.initialize_ros(node)
        ComputeCalibState.initialize_ros(node)
        OperatableStateMachine.initialize_ros(node)
        Logger.initialize(node)

        # [MANUAL_INIT]
        # [/MANUAL_INIT]

    def create(self):
        _state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
        
        # Variables to pass data between states
        _state_machine.userdata.base_h_tool_accumulated = None
        _state_machine.userdata.camera_h_charuco_accumulated = None

        # [MANUAL_CREATE]
        # [/MANUAL_CREATE]

        with _state_machine:
            # STATE 1: Process offline with charuco_hand_eye
            OperatableStateMachine.add('Process_Offline',
                OfflineFindCharucoState(
                    pictures_folder=self.pictures_folder,
                    robot_poses_folder=self.robot_poses_folder,
                    output_folder=self.charuco_output_folder,
                    eye_in_hand=self.eye_in_hand
                ),
                transitions={'completed': 'Compute_Calibration', 'failed': 'failed'},
                autonomy={'completed': Autonomy.Off, 'failed': Autonomy.Off},
                remapping={
                    'base_h_tool_accumulated': 'base_h_tool_accumulated',
                    'camera_h_charuco_accumulated': 'camera_h_charuco_accumulated'
                })

            # STATE 2: Compute calibration with VISP
            OperatableStateMachine.add('Compute_Calibration',
                ComputeCalibState(
                    eye_in_hand_mode=self.eye_in_hand,
                    calibration_file_name=self.calibration_file_name,
                    customize_file=True,
                    launch_visp=True,
                    output_folder=self.output_folder
                ),
                transitions={'finish': 'finished', 'failed': 'failed'},
                autonomy={'finish': Autonomy.Off, 'failed': Autonomy.Off},
                remapping={
                    'base_h_tool': 'base_h_tool_accumulated',
                    'camera_h_charuco': 'camera_h_charuco_accumulated'
                })

        return _state_machine

    # [MANUAL_FUNC]
    # [/MANUAL_FUNC]
