#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, Logger
from hand_eye_flexbe_states.launch_moveit import LaunchMoveItState
from hand_eye_flexbe_states.take_pose_and_picture import TakePoseAndPictureState
from hand_eye_flexbe_states.offline_find_charuco import OfflineFindCharucoState
from hand_eye_flexbe_states.compute_calib import ComputeCalibState
import os
# [MANUAL_IMPORT]
# [/MANUAL_IMPORT]


'''
Created on Thu Feb 26 2026
@author: Gabriel Novas
'''
class CaptureAndCalibrateSM(Behavior):
    """
    FULL Hand-Eye Calibration Behavior:
    
    PHASE 1: Launches MoveIt + RViz automatically
    PHASE 2: You control the robot using MoveIt interface
             Press SPACE at each pose to capture
    PHASE 3: Offline processing with charuco_hand_eye
    PHASE 4: Automatic calibration computation with VISP
    
    FINAL RESULTS:
    - /home/drims/drims_ws/calibrations/camera_extrinsics.yaml (main calibration)
    - /home/drims/drims_ws/calibrations/extrinsic_calibration/charuco_table_poses/ (intermediate detections)
    - /home/drims/drims_ws/calibrations/extrinsic_matrix.yaml (T_w2c and T_c2w matrices)
    """
    
    def __init__(self, node):
        super(CaptureAndCalibrateSM, self).__init__()
        self.name = 'Capture and Calibrate Hand-Eye'
        self.node = node

        # Behavior parameters
        self.add_parameter('total_poses', 20)
        self.add_parameter('camera_type', 'realsense')
        self.add_parameter('base_frame', 'base_link')
        self.add_parameter('tool_frame', 'tool0')
        self.add_parameter('eye_in_hand', False)
        self.add_parameter('calibration_file_name', 'camera_extrinsics.yaml')
        
        # MoveIt parameters for UR5e
        self.add_parameter('moveit_launch_file', 'ur_moveit.launch.py')
        self.add_parameter('robot_name', 'ur5e')
        self.add_parameter('moveit_config_package', 'ur_moveit_config')
        self.add_parameter('robot_ip', '192.168.1.101')
        self.add_parameter('use_fake_hardware', False)
        
        # UNIFIED PATHS - Everything in /home/drims/drims_ws/calibrations
        base_calib_path = '/home/drims/drims_ws/calibrations'
        self.add_parameter('pictures_folder', f'{base_calib_path}/extrinsic_calibration/pictures')
        self.add_parameter('robot_poses_folder', f'{base_calib_path}/extrinsic_calibration/robot_poses')
        self.add_parameter('charuco_output_folder', f'{base_calib_path}/extrinsic_calibration/charuco_table_poses')
        self.add_parameter('output_folder', base_calib_path)

        # Initialize states
        LaunchMoveItState.initialize_ros(node)
        TakePoseAndPictureState.initialize_ros(node)
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
            # STATE 1: Launch MoveIt
            OperatableStateMachine.add('Launch_MoveIt',
                LaunchMoveItState(
                    moveit_launch_file=self.moveit_launch_file,
                    robot_name=self.robot_name,
                    moveit_config_package=self.moveit_config_package,
                    robot_ip=self.robot_ip,
                    use_fake_hardware=self.use_fake_hardware
                ),
                transitions={'done': 'Capture_Poses', 'failed': 'failed'},
                autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

            # STATE 2: Capture poses (you with SPACE)
            OperatableStateMachine.add('Capture_Poses',
                TakePoseAndPictureState(
                    total_poses=self.total_poses,
                    camera_type=self.camera_type,
                    base_frame=self.base_frame,
                    tool_frame=self.tool_frame,
                    pictures_folder=self.pictures_folder,
                    robot_poses_folder=self.robot_poses_folder,
                    auto_capture=False
                ),
                transitions={'done': 'Process_Offline', 'failed': 'failed'},
                autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

            # STATE 3: Process offline with charuco_hand_eye
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

            # STATE 4: Compute calibration with VISP
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
