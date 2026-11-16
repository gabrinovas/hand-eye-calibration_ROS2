# Copyright 2026 Gabriel Novas
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""FlexBE state that starts the MoveIt of the robot, for the operator to move it with."""

import time

from charuco_calibrator.calibration_layout import robot_entry
from flexbe_core import EventState, Logger
from hand_eye_flexbe_states.managed_process import ManagedProcess, ros_lists
from hand_eye_flexbe_states.settings import processes


class LaunchMoveItState(EventState):
    """
    Start MoveIt with RViz for a robot, unless it is already running, and leave it running.

    What is launched for each robot is in charuco_calibrator/config/robots.yaml.

    -- robot_name         string  Robot model (ur5e, ufactory_lite6)
    -- robot_ip           string  Address of the real robot ('' with fake hardware)
    -- use_fake_hardware  bool    Simulated robot instead of the real one

    <= done    MoveIt launched (it may still be starting)
    <= failed  Not used: the capture that follows fails when the robot gives no pose
    """

    def __init__(self, robot_name, robot_ip, use_fake_hardware):
        super().__init__(outcomes=['done', 'failed'])
        moveit = robot_entry(robot_name)['moveit']
        fake = str(bool(use_fake_hardware)).lower()
        self.command = ['ros2', 'launch', moveit['package'],
                        moveit['launch_fake' if use_fake_hardware else 'launch']]
        self.command += [argument.format(use_fake_hardware=fake)
                         for argument in moveit['arguments']]
        if str(robot_ip).strip():  # the address of the installation, given by the behavior
            self.command.append(f'robot_ip:={str(robot_ip).strip()}')
        self.settings = processes('moveit')
        self.moveit = ManagedProcess(self.command, quiet=True)

    def on_start(self):
        """Launch MoveIt when the behavior starts."""
        if ros_lists('node', [self.settings['node_name']]):
            Logger.loginfo('MoveIt is already running')
            return
        Logger.loginfo('Launching MoveIt: ' + ' '.join(self.command))
        try:
            self.moveit.start()
        except OSError as error:
            Logger.logerr(f'MoveIt could not be launched: {error}')
            return
        time.sleep(self.settings['startup_wait_s'])
        Logger.loginfo('MoveIt launched: the capture goes on while it finishes starting')

    def execute(self, userdata):
        """Go on to the capture."""
        return 'done'
