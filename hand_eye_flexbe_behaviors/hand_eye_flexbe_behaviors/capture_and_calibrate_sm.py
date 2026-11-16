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
from charuco_calibrator.calibration_layout import CAMERA_FRAME, calibration_folders, robot_entry
from charuco_calibrator.robot_addresses import cell_address
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
    
    One calibration per camera, robot and cell. FINAL RESULTS, in
    ~/calibrations/<camera>/extrinsic_calibration/<robot>/<cell>/:
    - calibration_results/camera_extrinsics.yaml (main calibration: robot, cell, frames and transform)
    - calibration_results/extrinsic_matrix.yaml (T_w2c and T_c2w matrices)
    - charuco_table_poses/ (intermediate detections)
    """
    
    def __init__(self, node):
        super(CaptureAndCalibrateSM, self).__init__()
        self.name = 'Capture and Calibrate Hand-Eye'
        self.node = node

        # Behavior parameters
        self.add_parameter('total_poses', 20)
        self.add_parameter('camera_type', 'realsense_d435i')
        self.add_parameter('eye_in_hand', False)
        self.add_parameter('camera_intrinsics_file', 'latest')
        self.add_parameter('calibration_file_name', 'camera_extrinsics.yaml')
        
        # Robot parameters. cell: the cell the robot and the camera stand in (the calibration is stored per cell).
        # robot_ip: empty takes the address of the cell from the robot addresses file of the installation.
        self.add_parameter('robot_name', 'ur5e')
        self.add_parameter('cell', '')
        self.add_parameter('robot_ip', '')
        self.add_parameter('use_fake_hardware', False)

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
        # x:30 y:50, x:130 y:50
        _state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

        # MoveIt, frames and folders of the robot and the cell (charuco_calibrator.calibration_layout)
        robot = robot_entry(self.robot_name)
        self.moveit_config_package = robot['moveit_package']
        self.moveit_launch_file = robot['moveit_launch_fake' if self.use_fake_hardware else 'moveit_launch']
        self.base_frame, self.tool_frame = robot['base_frame'], robot['tool_frame']
        folders = calibration_folders(self.camera_type, self.robot_name, self.cell)
        self.output_folder = folders['root']
        self.pictures_folder = folders['pictures']
        self.robot_poses_folder = folders['robot_poses']
        self.charuco_output_folder = folders['charuco_table_poses']

        # The address of the real robot: the one given, else the one of the cell in the robot addresses file
        robot_ip = str(self.robot_ip).strip()
        if not robot_ip and not self.use_fake_hardware:
            robot_ip = cell_address(str(self.cell).strip())
        Logger.loginfo(f"Calibration of {self.robot_name} in the cell '{self.cell}': {self.output_folder}")

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
                    robot_ip=robot_ip,
                    use_fake_hardware=self.use_fake_hardware
                ),
                transitions={'done': 'Capture_Poses', 'failed': 'failed'},
                autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

            # STATE 2: Capture poses (you with SPACE)
            OperatableStateMachine.add('Capture_Poses',
                TakePoseAndPictureState(
                    total_poses=self.total_poses,
                    camera_type=self.camera_type,
                    robot_name=self.robot_name,
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
                    eye_in_hand=self.eye_in_hand,
                    robot_name=self.robot_name,
                    camera_intrinsics_file=self.camera_intrinsics_file,
                    camera_type=self.camera_type,
                    base_frame=self.base_frame
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
                    robot_name=self.robot_name,
                    customize_file=True,
                    launch_visp=True,
                    output_folder=self.output_folder,
                    cell=str(self.cell).strip(),
                    base_frame=self.base_frame,
                    camera_frame=CAMERA_FRAME
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
