# Copyright 2021 Chien shao-yu, Gabriel Novas
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""FlexBE state that computes a camera-robot calibration with the ViSP solver."""

import os
import time

from charuco_calibrator.results_io import write_result
from charuco_calibrator.transforms import euler_of, inverted
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyServiceCaller
from hand_eye_flexbe_states.managed_process import executable_running, ManagedProcess, ros_lists
from hand_eye_flexbe_states.settings import processes
from visp_hand2eye_calibration.msg import TransformArray
from visp_hand2eye_calibration.srv import ComputeEffectorCameraQuick


class ComputeCalibState(EventState):
    """
    Compute the pose of the camera from the pairs of a calibration and write the result.

    The ViSP solver is started when the behavior starts (unless it is already running) and
    stopped when it ends. The result says what it is the calibration of: robot, cell and frames.

    -- eye_in_hand_mode  bool    True: the camera is on the robot
    -- robot_name        string  Robot of the calibration (written in the result)
    -- output_folder     string  Folder of the calibration of the camera, robot and cell
    -- cell              string  Cell of the calibration (written in the result)
    -- base_frame        string  Frame of the robot the result is expressed in
    -- camera_frame      string  Frame of the camera of the result

    ># base_h_tool       TransformArray  Robot poses (base to tool)
    ># camera_h_charuco  TransformArray  Board poses (camera to board)

    <= finish  Calibration computed and written
    <= failed  There are no pairs, or the solver failed
    """

    def __init__(self, eye_in_hand_mode, robot_name, output_folder, cell, base_frame,
                 camera_frame):
        super().__init__(outcomes=['finish', 'failed'],
                         input_keys=['base_h_tool', 'camera_h_charuco'])
        self.about = {'robot_model': str(robot_name), 'cell': str(cell),
                      'base_frame': str(base_frame), 'camera_frame': str(camera_frame),
                      'eye_in_hand': bool(eye_in_hand_mode)}
        missing = [key for key, value in self.about.items() if value == '']
        if missing or not output_folder:
            raise ValueError('[Fail-Fast] ComputeCalibState needs output_folder and the robot, '
                             'cell and frames of the calibration; missing: '
                             f'{missing or "output_folder"}')
        self.folder = os.path.join(str(output_folder), 'calibration_results')
        self.settings = processes('visp')
        self.solver = ManagedProcess(['ros2', 'run', self.settings['package'],
                                      self.settings['executable']], quiet=False)
        self.client = None
        Logger.loginfo(f'The calibration will be written in {self.folder}')

    def on_start(self):
        """Start the solver when the behavior starts, unless it is already running."""
        ProxyServiceCaller.initialize(ComputeCalibState._node)
        if not self.solver_running():
            Logger.loginfo('Starting the ViSP solver')
            self.solver.start()
            waited = 0.0
            while waited < self.settings['start_timeout_s'] and not self.solver_running():
                if self.solver.ended():
                    Logger.logerr('The ViSP solver ended at start: its error is above')
                    break
                time.sleep(self.settings['poll_period_s'])
                waited += self.settings['poll_period_s']
        self.client = ProxyServiceCaller({self.settings['service']: ComputeEffectorCameraQuick})

    def solver_running(self):
        """Return whether the solver runs: its process exists or its service is listed."""
        return (executable_running(self.settings['executable'])
                or ros_lists('service', [self.settings['service']]))

    def execute(self, userdata):
        """Ask the solver for the calibration once its service is there, and write the result."""
        service = self.settings['service']
        if self.client is None or not self.client.is_available(service):
            if self.solver.ended():
                Logger.logerr('The ViSP solver is not running')
                return 'failed'
            Logger.logwarn('Waiting for the service of the ViSP solver')
            return None
        robot, board = userdata.base_h_tool, userdata.camera_h_charuco
        counts = [len(poses.transforms) if hasattr(poses, 'transforms') else 0
                  for poses in (robot, board)]
        if counts[0] == 0 or counts[0] != counts[1]:
            Logger.logerr(f'Nothing to calibrate with: {counts[0]} robot poses and {counts[1]} '
                          'board poses')
            return 'failed'
        request = ComputeEffectorCameraQuick.Request()
        request.camera_object, request.world_effector = TransformArray(), TransformArray()
        request.camera_object.header, request.world_effector.header = board.header, robot.header
        request.camera_object.transforms = board.transforms
        # with the camera fixed (eye to hand) the solver takes the robot poses inverted
        request.world_effector.transforms = (
            robot.transforms if self.about['eye_in_hand']
            else [inverted(pose) for pose in robot.transforms])
        Logger.loginfo(f'Calibrating with {counts[0]} poses')
        response = self.client.call(service, request)
        if response is None:
            Logger.logerr('The ViSP solver gave no answer')
            return 'failed'
        pose = response.effector_camera
        translation, rotation, euler = pose.translation, pose.rotation, euler_of(pose)
        Logger.loginfo(f'Camera pose: x {translation.x:.6f}, y {translation.y:.6f}, '
                       f'z {translation.z:.6f} m; quaternion x {rotation.x:.6f}, '
                       f'y {rotation.y:.6f}, z {rotation.z:.6f}, w {rotation.w:.6f}; '
                       f'roll {euler[0]:.6f}, pitch {euler[1]:.6f}, yaw {euler[2]:.6f} rad')
        written = write_result(self.folder, pose, dict(self.about, num_poses_used=counts[0]))
        Logger.loginfo(f'Written: {written}')
        return 'finish'

    def on_stop(self):
        """Stop the solver this state started."""
        self.solver.stop()
