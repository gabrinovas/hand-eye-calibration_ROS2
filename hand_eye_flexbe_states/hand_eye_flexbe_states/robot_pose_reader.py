# Copyright 2026 Gabriel Novas
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Pose of the robot when a picture is taken: the transform between two of its frames, from TF."""

from hand_eye_flexbe_states.settings import capture_ui
import rclpy
from rclpy.duration import Duration
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class RobotPoseReader:
    """Listens to the transforms of the robot and reads the one from its base to its tool."""

    def __init__(self, node, base_frame, tool_frame):
        self.base_frame, self.tool_frame = base_frame, tool_frame
        self.settings = capture_ui('hand_eye', 'robot_pose')
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, node, spin_thread=True)

    def read(self):
        """Return (geometry_msgs Transform, '') of the tool in the base frame, or (None, why)."""
        timeout = Duration(seconds=self.settings['lookup_timeout_s'])
        try:
            known = self.buffer.can_transform(self.base_frame, self.tool_frame, rclpy.time.Time(),
                                              timeout=timeout)
            if not known:
                frames = self.buffer.all_frames_as_string()[:self.settings['frames_shown_chars']]
                return None, (f"no transform from '{self.base_frame}' to '{self.tool_frame}' "
                              f'(frames known: {frames}...)')
            found = self.buffer.lookup_transform(self.base_frame, self.tool_frame,
                                                 rclpy.time.Time())
        except TransformException as error:
            return None, str(error)
        return found.transform, ''
