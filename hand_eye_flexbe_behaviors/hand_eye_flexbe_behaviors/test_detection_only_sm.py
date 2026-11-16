# Copyright 2026 Gabriel Novas
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

# This file keeps the layout the FlexBE editor generates, because the FlexBE app reads it to show
# and run the behavior. It is maintained by hand: saving the behavior from the editor writes the
# file again and keeps only the code inside the [MANUAL] tags.

"""Behavior Test Detection Only: a camera-robot calibration computed again from what is stored."""

# [MANUAL_IMPORT]
from charuco_calibrator.calibration_layout import calibration_folders
from charuco_calibrator.calibration_layout import camera_frame
from charuco_calibrator.calibration_layout import checked_cell
from charuco_calibrator.calibration_layout import layout
from charuco_calibrator.calibration_layout import robot_entry
# [/MANUAL_IMPORT]
from flexbe_core import Autonomy, Behavior, Logger, OperatableStateMachine
from hand_eye_flexbe_states.compute_calib import ComputeCalibState
from hand_eye_flexbe_states.offline_find_charuco import OfflineFindCharucoState


class TestDetectionOnlySM(Behavior):
    """
    Compute a camera-robot calibration again from its stored pictures and robot poses.

    It needs no robot and no camera: only the pictures and robot_poses folders of the camera,
    robot and cell, and the intrinsic calibration in use of the camera.

    1. The board is detected in the stored pictures (node charuco_hand_eye).
    2. The ViSP solver computes the pose of the camera (calibration_results).
    """

    def __init__(self, node):
        super(TestDetectionOnlySM, self).__init__()
        self.name = 'Test Detection Only'
        self.node = node

        # parameters of this behavior: the same, with the same defaults, as its manifest
        self.add_parameter('camera_type', 'realsense_d435i')
        self.add_parameter('robot_name', 'ur5e')
        self.add_parameter('cell', '')
        self.add_parameter('eye_in_hand', False)

        OfflineFindCharucoState.initialize_ros(node)
        ComputeCalibState.initialize_ros(node)
        OperatableStateMachine.initialize_ros(node)
        Logger.initialize(node)

        # [MANUAL_INIT]
        # [/MANUAL_INIT]

    def create(self):
        # x:30 y:50, x:130 y:50
        _state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
        _state_machine.userdata.base_h_tool_accumulated = None
        _state_machine.userdata.camera_h_charuco_accumulated = None

        # [MANUAL_CREATE]
        # frames and folders of the robot and the cell
        cell = checked_cell(self.cell)
        folders = calibration_folders(self.camera_type, self.robot_name, cell)
        base_frame = robot_entry(self.robot_name)['base_frame']
        pictures_folder, poses_folder = folders['pictures'], folders['robot_poses']
        pairs_folder, calibration_folder = folders['charuco_table_poses'], folders['root']
        camera, intrinsics_in_use = camera_frame(), layout('intrinsic', 'latest')
        Logger.loginfo(f"Calibration of {self.robot_name} in the cell '{cell}': "
                       f'{calibration_folder}')
        # [/MANUAL_CREATE]

        with _state_machine:
            # x:30 y:150
            OperatableStateMachine.add('Process_Offline',
                                       OfflineFindCharucoState(
                                           pictures_folder=pictures_folder,
                                           robot_poses_folder=poses_folder,
                                           output_folder=pairs_folder,
                                           eye_in_hand=self.eye_in_hand,
                                           camera_intrinsics_file=intrinsics_in_use,
                                           camera_type=self.camera_type,
                                           base_frame=base_frame),
                                       transitions={'completed': 'Compute_Calibration',
                                                    'failed': 'failed'},
                                       autonomy={'completed': Autonomy.Off,
                                                 'failed': Autonomy.Off},
                                       remapping={'base_h_tool_accumulated':
                                                  'base_h_tool_accumulated',
                                                  'camera_h_charuco_accumulated':
                                                  'camera_h_charuco_accumulated'})

            # x:230 y:150
            OperatableStateMachine.add('Compute_Calibration',
                                       ComputeCalibState(
                                           eye_in_hand_mode=self.eye_in_hand,
                                           robot_name=self.robot_name,
                                           output_folder=calibration_folder,
                                           cell=cell,
                                           base_frame=base_frame,
                                           camera_frame=camera),
                                       transitions={'finish': 'finished', 'failed': 'failed'},
                                       autonomy={'finish': Autonomy.Off, 'failed': Autonomy.Off},
                                       remapping={'base_h_tool': 'base_h_tool_accumulated',
                                                  'camera_h_charuco':
                                                  'camera_h_charuco_accumulated'})

        return _state_machine

    # [MANUAL_FUNC]
    # [/MANUAL_FUNC]
