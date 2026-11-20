# Copyright 2026 Gabriel Novas
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""
Color pictures from a camera of charuco_calibrator/config/cameras.yaml: RealSense or USB. A
RealSense camera is opened by its serial number, so that the pictures are always of the unit whose
calibration is being made.

pyrealsense2 is imported when a RealSense camera is started, so that the states can be loaded on a
computer without it.
"""

import time

from charuco_calibrator.config_loader import ConfigError, setting
import cv2
import numpy as np

REALSENSE, USB = 'realsense', 'usb'


class CameraStream:
    """The color stream of a camera: start it, take pictures from it and stop it."""

    def __init__(self, camera_type, serial):
        self.serial = str(serial)
        cameras = setting('cameras', 'cameras')
        if camera_type not in cameras:
            raise ConfigError(f"[Fail-Fast] Unknown camera '{camera_type}': "
                              f'charuco_calibrator/config/cameras.yaml has {sorted(cameras)}')
        self.settings = cameras[camera_type]
        if self.settings['driver'] not in (REALSENSE, USB):
            raise ConfigError(f"[Fail-Fast] The driver '{self.settings['driver']}' of the camera "
                              f"'{camera_type}' is neither '{REALSENSE}' nor '{USB}'")
        self.pipeline = self.capture = None

    def start(self, warm_up_s):
        """
        Start the camera with its 'color' stream; return that stream as text.

        A camera that cannot give exactly that stream is a RuntimeError: no other resolution is
        tried, because the calibrations are valid for one resolution only.
        """
        stream = self._open_usb() if self.settings['driver'] == USB else self._open_realsense()
        time.sleep(warm_up_s)
        return stream

    def _open_realsense(self):
        import pyrealsense2 as rs
        color = self.settings['color']
        asked = f"{color['width']}x{color['height']} at {color['fps']} Hz"
        pipeline, config = rs.pipeline(), rs.config()
        config.enable_device(self.serial)
        config.enable_stream(rs.stream.color, color['width'], color['height'], rs.format.bgr8,
                             color['fps'])
        try:
            pipeline.start(config)
        except RuntimeError as error:
            raise RuntimeError(f'[Fail-Fast] The RealSense camera {self.serial} does not give its '
                               f'stream of cameras.yaml ({asked}): {error}') from error
        self.pipeline = pipeline
        return f'RealSense {self.serial} {asked}'

    def _open_usb(self):
        capture, color = cv2.VideoCapture(self.settings['device']), self.settings['color']
        if not capture.isOpened():
            raise RuntimeError(f"The USB camera {self.settings['device']} could not be opened")
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, color['width'])
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, color['height'])
        given = (int(capture.get(cv2.CAP_PROP_FRAME_WIDTH)),
                 int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT)))
        if given != (color['width'], color['height']) or not capture.read()[0]:
            capture.release()
            asked = f"{color['width']}x{color['height']}"
            raise RuntimeError(f"[Fail-Fast] The USB camera {self.settings['device']} does not "
                               f'give its stream of cameras.yaml ({asked}): it gives '
                               f'{given[0]}x{given[1]}')
        self.capture = capture
        return f'USB camera {given[0]}x{given[1]}'

    def picture(self):
        """Return the next picture, or None when there is none yet. RuntimeError on failure."""
        if self.pipeline is not None:
            frames = self.pipeline.wait_for_frames(timeout_ms=self.settings['frame_timeout_ms'])
            color = frames.get_color_frame()
            return np.asanyarray(color.get_data()) if color else None
        if self.capture is None:
            raise RuntimeError('The camera has not been started')
        read, image = self.capture.read()
        if not read:
            raise RuntimeError('The USB camera gave no picture')
        return image

    def stop(self):
        """Stop the camera."""
        if self.pipeline is not None:
            self.pipeline.stop()
        if self.capture is not None:
            self.capture.release()
        self.pipeline = self.capture = None
