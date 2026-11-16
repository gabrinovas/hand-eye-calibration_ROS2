# Copyright 2026 Gabriel Novas
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""The state that waits for the pairs: what it listens to, and that it lets go of it."""

import threading
import time
from types import SimpleNamespace

from geometry_msgs.msg import Transform
from hand_eye_flexbe_states import offline_find_charuco
from hand_eye_flexbe_states.offline_find_charuco import OfflineFindCharucoState
import pytest
import rclpy
from rclpy.executors import MultiThreadedExecutor
from visp_hand2eye_calibration.msg import TransformArray

TOPICS = ('/world_effector_poses', '/camera_object_poses')
GRAPH_WAIT_S = 3.0  # the ROS graph takes a moment to show a subscription that came or went


@pytest.fixture(scope='module')
def node():
    rclpy.init()
    node = rclpy.create_node('detection_state_test')
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()
    OfflineFindCharucoState.initialize_ros(node)
    yield node
    rclpy.shutdown()


@pytest.fixture
def state(node, tmp_path, monkeypatch):
    """Return the state with a detector that is taken as already running (none is started)."""
    monkeypatch.setattr(offline_find_charuco, 'ros_lists', lambda kind, names: True)
    return OfflineFindCharucoState(
        pictures_folder=str(tmp_path / 'pictures'), robot_poses_folder=str(tmp_path / 'poses'),
        output_folder=str(tmp_path / 'pairs'), eye_in_hand=False, camera_intrinsics_file='latest',
        camera_type='realsense_d435i', base_frame='base_link')


def subscribers(node, expected):
    """Return the subscribers of each topic, once the ROS graph shows the expected ones."""
    end = time.time() + GRAPH_WAIT_S
    while time.time() < end:
        counts = [node.count_subscribers(topic) for topic in TOPICS]
        if counts == expected:
            break
        time.sleep(0.05)
    return counts


def test_entering_twice_keeps_two_subscriptions_and_leaving_releases_them(node, state):
    state.on_enter(SimpleNamespace())
    first = list(state.subscriptions)
    state.on_enter(SimpleNamespace())  # entered again
    assert len(state.subscriptions) == 2 and not set(first) & set(state.subscriptions)
    assert subscribers(node, [1, 1]) == [1, 1]  # one per topic, not two
    state.on_exit(SimpleNamespace())
    assert state.subscriptions == [] and subscribers(node, [0, 0]) == [0, 0]
    state.on_stop()  # stopping after leaving is not an error
    assert not state.detector.started()


def test_the_pairs_are_taken_when_both_lists_hold_the_same_poses(state):
    data = SimpleNamespace(base_h_tool_accumulated=None, camera_h_charuco_accumulated=None)
    state.on_enter(data)
    robot, board = TransformArray(), TransformArray()
    robot.transforms, board.transforms = [Transform()] * 3, [Transform()] * 2
    state.received = {'base_h_tool_accumulated': robot}
    assert state.pairs() is None  # one list only
    state.received['camera_h_charuco_accumulated'] = board
    assert state.pairs() is None  # not the same number of poses
    board.transforms = [Transform()] * 3
    assert state.pairs() is not None
    assert '-p' in state.command and 'publish_rate:=0.5' in state.command
    state.on_stop()
