# Copyright 2026 Gabriel Novas
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""FlexBE state that gets the pairs of a calibration from its stored pictures and robot poses."""

import os
import time

from charuco_calibrator.config_loader import setting
from flexbe_core import EventState, Logger
from hand_eye_flexbe_states.managed_process import ManagedProcess, ros_lists
from hand_eye_flexbe_states.settings import processes
from visp_hand2eye_calibration.msg import TransformArray


class OfflineFindCharucoState(EventState):
    """
    Run the node charuco_hand_eye on the stored pictures and poses and take the pairs it publishes.

    The node detects the board in every picture, joins it with the robot pose of the same number,
    stores the pairs in the output folder and publishes them. It is stopped when the behavior ends.

    -- pictures_folder         string  Folder with the pictures of the calibration
    -- robot_poses_folder      string  Folder with the robot poses of the calibration
    -- output_folder           string  Folder the pairs and the drawn pictures are stored in
    -- eye_in_hand             bool    True: the camera is on the robot
    -- camera_intrinsics_file  string  Intrinsic result to use: 'latest' or one of its files
    -- camera_type             string  Camera of the calibration
    -- camera_serial           string  Serial number of the camera unit of the pictures
    -- base_frame              string  Frame of the robot the poses are expressed in

    #> base_h_tool_accumulated       TransformArray  Robot poses (base to tool)
    #> camera_h_charuco_accumulated  TransformArray  Board poses (camera to board)

    <= completed  The pairs were received
    <= failed     The node ended or the pairs did not arrive in time
    """

    def __init__(self, pictures_folder, robot_poses_folder, output_folder, eye_in_hand,
                 camera_intrinsics_file, camera_type, camera_serial, base_frame):
        super().__init__(outcomes=['completed', 'failed'],
                         output_keys=['base_h_tool_accumulated', 'camera_h_charuco_accumulated'])
        self.settings = processes('detector')
        self.output_folder = str(output_folder)
        self.command = [
            'ros2', 'run', self.settings['package'], self.settings['executable'], '--ros-args',
            '-p', f'pictures_folder:={pictures_folder}',
            '-p', f'robot_poses_folder:={robot_poses_folder}',
            '-p', f'output_folder:={self.output_folder}',
            '-p', f'eye_in_hand:={str(bool(eye_in_hand)).lower()}',
            '-p', f'base_frame:={base_frame}',
            '-p', f'camera_name:={camera_type}',
            '-p', f"camera_serial:='{camera_serial}'",  # quoted: a number would not be text
            '-p', f'camera_intrinsics_file:={camera_intrinsics_file}',
            '-p', f"publish_rate:={float(self.settings['publish_rate_hz'])}"]
        self.detector = ManagedProcess(self.command, quiet=False)
        self.subscriptions, self.received, self.started_at = [], {}, None
        Logger.loginfo(f'Pictures {pictures_folder} and robot poses {robot_poses_folder}: their '
                       f'pairs go to {self.output_folder}')

    def on_enter(self, userdata):
        """Start the node, unless one is already running, and listen to what it publishes."""
        self.received, self.started_at = {}, time.time()
        self.stop_listening()  # entered again: the subscriptions of before are not kept
        os.makedirs(self.output_folder, exist_ok=True)
        publication = setting('hand_eye_node')
        for key, topic in (('base_h_tool_accumulated', publication['topics']['robot_poses']),
                           ('camera_h_charuco_accumulated', publication['topics']['board_poses'])):
            self.subscriptions.append(self._node.create_subscription(
                TransformArray, topic,
                lambda message, key=key: self.received.update({key: message}),
                publication['qos_depth']))
        if ros_lists('node', self.settings['node_names']):
            Logger.loginfo('The node charuco_hand_eye is already running')
            return
        Logger.loginfo('Starting the board detection: ' + ' '.join(self.command))
        self.detector.start()
        time.sleep(self.settings['startup_wait_s'])

    def pairs(self):
        """Return the two lists received when they hold the same poses, else None."""
        lists = [self.received[key] for key in sorted(self.received)]
        complete = len(lists) == 2 and len(lists[0].transforms) == len(lists[1].transforms) > 0
        return self.received if complete else None

    def execute(self, userdata):
        """Wait for the pairs; fail when the node ends without them or they take too long."""
        pairs = self.pairs()
        if pairs is not None:
            userdata.base_h_tool_accumulated = pairs['base_h_tool_accumulated']
            userdata.camera_h_charuco_accumulated = pairs['camera_h_charuco_accumulated']
            Logger.loginfo(f"{len(pairs['base_h_tool_accumulated'].transforms)} pairs received")
            return 'completed'
        if self.detector.ended():
            Logger.logerr('The board detection ended without pairs: its error is above')
            return 'failed'
        if time.time() - self.started_at > self.settings['detection_timeout_s']:
            Logger.logerr(f"No pairs after {self.settings['detection_timeout_s']} s")
            return 'failed'
        return None

    def stop_listening(self):
        """Release the subscriptions of this state."""
        for subscription in self.subscriptions:
            self._node.destroy_subscription(subscription)
        self.subscriptions = []

    def on_exit(self, userdata):
        """Stop listening: the pairs have been taken (the node goes on until the behavior ends)."""
        self.stop_listening()

    def on_stop(self):
        """Stop listening and stop the node this state started."""
        self.stop_listening()
        self.detector.stop()
