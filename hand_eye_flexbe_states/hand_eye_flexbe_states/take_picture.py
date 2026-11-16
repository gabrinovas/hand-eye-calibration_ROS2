# Copyright 2021 Chien shao-yu, Gabriel Novas
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""FlexBE state that takes the pictures of the board for the intrinsic calibration of a camera."""

import glob
import os

from charuco_calibrator.calibration_layout import INTRINSIC, layout
import cv2
from flexbe_core import EventState, Logger
from hand_eye_flexbe_states.camera_stream import CameraStream
from hand_eye_flexbe_states.capture_window import CANCEL, CAPTURE, CaptureWindow
from hand_eye_flexbe_states.settings import capture_ui

SECTION = 'intrinsic'


class TakePictureState(EventState):
    """
    Take pictures of the board with the camera: the operator presses SPACE or ENTER for each one.

    When the behavior starts, the pictures of a previous capture are deleted from the folder.

    -- pic_num        int     Number of pictures to take
    -- camera_type    string  Camera (charuco_calibrator/config/cameras.yaml)
    -- output_folder  string  Folder the pictures are saved in

    <= done    All the pictures were taken
    <= failed  The camera failed or the operator cancelled (ESC)
    """

    def __init__(self, pic_num, camera_type, output_folder):
        super(TakePictureState, self).__init__(outcomes=['done', 'failed'])
        self.total, self.taken = int(pic_num), 0
        self.folder = str(output_folder)
        self.camera, self.window = CameraStream(camera_type), CaptureWindow(SECTION)
        self.ready = False
        os.makedirs(self.folder, exist_ok=True)
        Logger.loginfo(f'{self.total} pictures will be saved in {self.folder}')

    def on_start(self):
        """Delete the pictures of a previous capture and start the camera."""
        old = [path for pattern in layout(INTRINSIC, 'capture_patterns')
               for path in glob.glob(os.path.join(self.folder, pattern))]
        for path in old:
            os.remove(path)
        Logger.loginfo(f'{len(old)} pictures of a previous capture deleted')
        look = capture_ui(SECTION)
        try:
            stream = self.camera.start(look['warm_up_s'])
            Logger.loginfo(f'Camera started: {stream}')
            self.ready = True
        except RuntimeError as error:
            Logger.logerr(f'The camera could not be started: {error}')

    def execute(self, userdata):
        """Show the camera and save a picture when the operator asks, until all are taken."""
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
                                 remaining=self.total - self.taken,
                                 folder=os.path.basename(self.folder))
        if asked == CANCEL:
            Logger.logwarn('Capture cancelled')
            return self.end('failed')
        if asked == CAPTURE:
            name = layout(INTRINSIC, 'picture').format(index=self.taken + 1)
            cv2.imwrite(os.path.join(self.folder, name), picture)
            self.taken += 1
            Logger.loginfo(f'Picture {self.taken}/{self.total} saved: {name}')
            if self.taken >= self.total:
                return self.end('done')
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
