# Copyright 2021 Chien shao-yu, Gabriel Novas
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

# This file keeps the layout the FlexBE editor generates, because the FlexBE app reads it to show
# and run the behavior. It is maintained by hand: saving the behavior from the editor writes the
# file again and keeps only the code inside the [MANUAL] tags.

"""Behavior camera_calibration: intrinsic calibration of a camera with pictures of the board."""

# [MANUAL_IMPORT]
from charuco_calibrator.calibration_layout import intrinsic_folders
# [/MANUAL_IMPORT]
from flexbe_core import Autonomy, Behavior, Logger, OperatableStateMachine
from hand_eye_flexbe_states.charuco_camera_calibration import CharucoCameraCalibrationState
from hand_eye_flexbe_states.take_picture import TakePictureState


class camera_calibrationSM(Behavior):
    """
    Calibrate a camera with pictures of the ChArUco board.

    1. The operator takes the pictures of the board (SPACE or ENTER for each one, ESC cancels).
    2. The camera is calibrated with them. The result is written in the intrinsic folder of the
       camera: camera_intrinsics.yaml (the one in use) and a dated copy.
    """

    def __init__(self, node):
        super(camera_calibrationSM, self).__init__()
        self.name = 'camera_calibration'
        self.node = node

        # parameters of this behavior: the same, with the same defaults, as its manifest
        self.add_parameter('pic_num', 30)
        self.add_parameter('camera_type', 'realsense_d435i')

        CharucoCameraCalibrationState.initialize_ros(node)
        TakePictureState.initialize_ros(node)
        OperatableStateMachine.initialize_ros(node)
        Logger.initialize(node)

        # [MANUAL_INIT]
        # [/MANUAL_INIT]

    def create(self):
        # x:570 y:120, x:130 y:365
        _state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

        # [MANUAL_CREATE]
        pictures_folder = intrinsic_folders(self.camera_type)['pictures']
        # [/MANUAL_CREATE]

        with _state_machine:
            # x:149 y:88
            OperatableStateMachine.add('take_camera_cali_pic',
                                       TakePictureState(pic_num=self.pic_num,
                                                        camera_type=self.camera_type,
                                                        output_folder=pictures_folder),
                                       transitions={'done': 'camera_calibration',
                                                    'failed': 'failed'},
                                       autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

            # x:348 y:87
            OperatableStateMachine.add('camera_calibration',
                                       CharucoCameraCalibrationState(
                                           images_folder=pictures_folder,
                                           camera_type=self.camera_type),
                                       transitions={'done': 'finished', 'failed': 'failed'},
                                       autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

        return _state_machine

    # [MANUAL_FUNC]
    # [/MANUAL_FUNC]
