# Copyright 2026 Gabriel Novas
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

# This file keeps the layout the FlexBE editor generates, because the FlexBE app reads it to show
# and run the behavior. It is maintained by hand: saving the behavior from the editor writes the
# file again and keeps only the code inside the [MANUAL] tags.

"""Behavior Capture and Calibrate Hand-Eye: camera-robot calibration of a robot in a cell."""

# [MANUAL_IMPORT]
from charuco_calibrator.calibration_layout import calibration_folders
from charuco_calibrator.calibration_layout import camera_frame
from charuco_calibrator.calibration_layout import checked_cell
from charuco_calibrator.camera_serials import chosen_serial
from charuco_calibrator.calibration_layout import robot_entry
from charuco_calibrator.robot_addresses import cell_address
# [/MANUAL_IMPORT]
from flexbe_core import Autonomy, Behavior, Logger, OperatableStateMachine
from hand_eye_flexbe_states.compute_calib import ComputeCalibState
from hand_eye_flexbe_states.launch_moveit import LaunchMoveItState
from hand_eye_flexbe_states.offline_find_charuco import OfflineFindCharucoState
from hand_eye_flexbe_states.take_pose_and_picture import TakePoseAndPictureState


class CaptureAndCalibrateSM(Behavior):
    """
    Calibrate the camera of a cell to its robot, capturing the pictures and the robot poses.

    1. MoveIt and RViz are launched.
    2. The operator moves the robot with MoveIt and presses SPACE at each pose to capture.
    3. The board is detected in the pictures (node charuco_hand_eye).
    4. The ViSP solver computes the pose of the camera.

    One calibration per camera, robot and cell, stored in
    <calibrations>/<camera>/extrinsic_calibration/<robot>/<cell>/: what was captured (pictures,
    robot_poses), the detections (charuco_table_poses) and the result (calibration_results).
    """

    def __init__(self, node):
        super(CaptureAndCalibrateSM, self).__init__()
        self.name = 'Capture and Calibrate Hand-Eye'
        self.node = node

        # parameters of this behavior: the same, with the same defaults, as its manifest
        self.add_parameter('total_poses', 20)
        self.add_parameter('camera_type', 'realsense_d435i')
        self.add_parameter('camera_serial', '')
        self.add_parameter('robot_name', 'ur5e')
        self.add_parameter('cell', '')
        self.add_parameter('robot_ip', '')
        self.add_parameter('camera_intrinsics_file', 'latest')
        self.add_parameter('eye_in_hand', False)
        self.add_parameter('use_fake_hardware', False)

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
        _state_machine.userdata.base_h_tool_accumulated = None
        _state_machine.userdata.camera_h_charuco_accumulated = None

        # [MANUAL_CREATE]
        # frames and folders of the robot and the cell, and the address of the real robot: the one
        # given, else the one of the cell in the robot addresses file of the installation
        robot = robot_entry(self.robot_name)
        cell = checked_cell(self.cell)
        folders = calibration_folders(self.camera_type, self.robot_name, cell)
        serial = chosen_serial(self.camera_type, self.camera_serial)
        base_frame, tool_frame = robot['base_frame'], robot['tool_frame']
        pictures_folder, poses_folder = folders['pictures'], folders['robot_poses']
        pairs_folder, calibration_folder = folders['charuco_table_poses'], folders['root']
        camera = camera_frame()
        robot_ip = str(self.robot_ip).strip()
        if not robot_ip and not self.use_fake_hardware:
            robot_ip = cell_address(cell)
        Logger.loginfo(f"Calibration of {self.robot_name} in the cell '{cell}': "
                       f'{calibration_folder}')
        # [/MANUAL_CREATE]

        with _state_machine:
            # x:30 y:150
            OperatableStateMachine.add('Launch_MoveIt',
                                       LaunchMoveItState(
                                           robot_name=self.robot_name,
                                           robot_ip=robot_ip,
                                           use_fake_hardware=self.use_fake_hardware),
                                       transitions={'done': 'Capture_Poses', 'failed': 'failed'},
                                       autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

            # x:230 y:150
            OperatableStateMachine.add('Capture_Poses',
                                       TakePoseAndPictureState(
                                           total_poses=self.total_poses,
                                           camera_type=self.camera_type,
                                           camera_serial=serial,
                                           robot_name=self.robot_name,
                                           base_frame=base_frame,
                                           tool_frame=tool_frame,
                                           pictures_folder=pictures_folder,
                                           robot_poses_folder=poses_folder),
                                       transitions={'done': 'Process_Offline', 'failed': 'failed'},
                                       autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

            # x:430 y:150
            OperatableStateMachine.add('Process_Offline',
                                       OfflineFindCharucoState(
                                           pictures_folder=pictures_folder,
                                           robot_poses_folder=poses_folder,
                                           output_folder=pairs_folder,
                                           eye_in_hand=self.eye_in_hand,
                                           camera_intrinsics_file=self.camera_intrinsics_file,
                                           camera_type=self.camera_type,
                                           camera_serial=serial,
                                           base_frame=base_frame),
                                       transitions={'completed': 'Compute_Calibration',
                                                    'failed': 'failed'},
                                       autonomy={'completed': Autonomy.Off,
                                                 'failed': Autonomy.Off},
                                       remapping={'base_h_tool_accumulated':
                                                  'base_h_tool_accumulated',
                                                  'camera_h_charuco_accumulated':
                                                  'camera_h_charuco_accumulated'})

            # x:630 y:150
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
