# Copyright 2026 Gabriel Novas
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""FlexBE state that takes the pictures and the robot poses of a camera-robot calibration."""

import glob
import os
import time

from charuco_calibrator.calibration_layout import EXTRINSIC, layout
from charuco_calibrator.poses_io import write_pose
import cv2
from flexbe_core import EventState, Logger
from hand_eye_flexbe_states.camera_stream import CameraStream
from hand_eye_flexbe_states.capture_window import CANCEL, CAPTURE, CaptureWindow
from hand_eye_flexbe_states.robot_pose_reader import RobotPoseReader
from hand_eye_flexbe_states.settings import capture_ui

SECTION = 'hand_eye'


class TakePoseAndPictureState(EventState):
    """
    Take a picture and the pose of the robot each time the operator presses SPACE or ENTER.

    When the behavior starts, the pictures and poses of a previous capture are deleted.

    -- total_poses         int     Number of poses to capture
    -- camera_type         string  Camera (charuco_calibrator/config/cameras.yaml)
    -- robot_name          string  Robot, shown in the window
    -- base_frame          string  Frame of the robot the poses are expressed in
    -- tool_frame          string  Frame of the robot whose pose is stored
    -- pictures_folder     string  Folder the pictures are saved in
    -- robot_poses_folder  string  Folder the poses are saved in

    <= done    All the poses were captured
    <= failed  The camera failed or the operator cancelled (ESC)
    """

    def __init__(self, total_poses, camera_type, robot_name, base_frame, tool_frame,
                 pictures_folder, robot_poses_folder):
        super().__init__(outcomes=['done', 'failed'])
        self.total, self.taken = int(total_poses), 0
        self.base_frame, self.tool_frame = str(base_frame), str(tool_frame)
        self.folders = {'pictures': str(pictures_folder), 'robot_poses': str(robot_poses_folder)}
        for folder in self.folders.values():
            os.makedirs(folder, exist_ok=True)
        self.camera = CameraStream(camera_type)
        self.window = CaptureWindow(SECTION, robot=str(robot_name).upper())
        self.poses, self.ready = None, False
        Logger.loginfo(f'{self.total} poses will be captured: pictures in '
                       f"{self.folders['pictures']}, poses ({self.base_frame} to "
                       f"{self.tool_frame}) in {self.folders['robot_poses']}")

    def on_start(self):
        """Delete what a previous capture left, listen to the robot and start the camera."""
        for kind, patterns in layout(EXTRINSIC, 'capture_patterns').items():
            for pattern in patterns:
                for path in glob.glob(os.path.join(self.folders[kind], pattern)):
                    os.remove(path)
        look = capture_ui(SECTION)
        self.poses = RobotPoseReader(self._node, self.base_frame, self.tool_frame)
        time.sleep(look['robot_pose']['listen_wait_s'])
        try:
            stream = self.camera.start(look['warm_up_s'])
            Logger.loginfo(f'Camera started: {stream}')
            self.ready = True
        except RuntimeError as error:
            Logger.logerr(f'The camera could not be started: {error}')

    def capture(self, picture):
        """Save the picture and the pose of the robot now; without the pose, save nothing."""
        pose, why = self.poses.read()
        if pose is None:
            Logger.logwarn(f'Not captured, the pose of the robot is not known: {why}')
            return
        index = self.taken + 1
        name = layout(EXTRINSIC, 'picture').format(index=index)
        cv2.imwrite(os.path.join(self.folders['pictures'], name), picture)
        stored = write_pose(self.folders['robot_poses'], index, self.base_frame, self.tool_frame,
                            pose)
        self.taken = index
        Logger.loginfo(f'Capture {self.taken}/{self.total}: {name} and {os.path.basename(stored)}')

    def execute(self, userdata):
        """Show the camera and capture when the operator asks, until all the poses are taken."""
        if self.taken >= self.total:
            return self.end('done')
        if not self.ready:
            return 'failed'
        try:
            picture = self.camera.picture()
        except RuntimeError as error:
            Logger.logerr(f'Camera error: {error}')
            return self.end('failed')
        if picture is None:
            return None
        asked = self.window.show(picture, taken=self.taken, total=self.total,
                                 remaining=self.total - self.taken)
        if asked == CANCEL:
            Logger.logwarn('Capture cancelled')
            return self.end('failed')
        if asked == CAPTURE:
            self.capture(picture)
        return None

    def end(self, outcome):
        """Close the window and the camera; return the outcome."""
        self.window.close()
        self.camera.stop()
        self.ready = False
        return outcome

    def on_stop(self):
        self.end(None)

    def on_exit(self, userdata):
        self.end(None)
