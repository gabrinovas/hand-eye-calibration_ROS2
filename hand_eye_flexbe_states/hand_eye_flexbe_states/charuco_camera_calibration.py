# Copyright 2021 Chien shao-yu, Gabriel Novas
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""FlexBE state that computes the intrinsic calibration of a camera from pictures of the board."""

from charuco_calibrator import intrinsics_io
from charuco_calibrator.board import load_board
from charuco_calibrator.config_loader import setting
from charuco_calibrator.intrinsic_solver import calibrate, picture_paths
import cv2
from flexbe_core import EventState, Logger


class CharucoCameraCalibrationState(EventState):
    """
    Calibrate a camera with the pictures of the board stored in a folder.

    The board is the one of charuco_calibrator/config/board.yaml. The result is written in the
    intrinsic folder of the camera unit as the calibration in use, with a dated copy as history.

    -- images_folder  string  Folder with the pictures of the board
    -- camera_type    string  Camera the result belongs to
    -- camera_serial  string  Serial number of the camera unit the result belongs to

    <= done    Calibration computed and written
    <= failed  Too few pictures could be used, or the calibration failed
    """

    def __init__(self, images_folder, camera_type, camera_serial):
        super(CharucoCameraCalibrationState, self).__init__(outcomes=['done', 'failed'])
        self.folder, self.camera_type = str(images_folder), str(camera_type)
        self.serial = str(camera_serial)
        self.board = load_board()
        Logger.loginfo(f'Pictures of the board ({self.board.columns} x {self.board.rows} squares '
                       f'of {self.board.square_length} m): {self.folder}')

    def execute(self, userdata):
        """Calibrate the camera and write the result."""
        paths = picture_paths(self.folder)
        if not paths:
            Logger.logerr(f'No pictures found in {self.folder}')
            return 'failed'
        Logger.loginfo(f'Processing {len(paths)} pictures')
        try:
            solution = calibrate(paths, self.board, setting('detection', 'intrinsic'),
                                 Logger.loginfo)
        except cv2.error as error:
            Logger.logerr(f'The calibration failed: {error}')
            return 'failed'
        if solution is None:
            return 'failed'
        Logger.loginfo(f'Camera calibrated with {solution.pictures_used} pictures, reprojection '
                       f'error {solution.error:.6f}')
        Logger.loginfo(f'Camera matrix:\n{solution.camera_matrix}')
        Logger.loginfo(f'Distortion coefficients: {solution.distortion.reshape(-1)}')
        for written in intrinsics_io.write(self.camera_type, self.serial, self.board,
                                           solution.error, solution.camera_matrix,
                                           solution.distortion, solution.image_size):
            Logger.loginfo(f'Written: {written}')
        return 'done'
