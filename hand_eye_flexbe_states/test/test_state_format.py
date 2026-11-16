# Copyright 2026 Gabriel Novas
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""The states keep the format the FlexBE app reads them with (it parses their source text)."""

import importlib
import inspect
import re

from flexbe_core import EventState
import pytest

STATES = {'charuco_camera_calibration': 'CharucoCameraCalibrationState',
          'compute_calib': 'ComputeCalibState', 'launch_moveit': 'LaunchMoveItState',
          'offline_find_charuco': 'OfflineFindCharucoState', 'take_picture': 'TakePictureState',
          'take_pose_and_picture': 'TakePoseAndPictureState'}
HELPERS = ('camera_stream', 'capture_window', 'managed_process', 'robot_pose_reader', 'settings')


def listed(source, name):
    """Return the quoted strings of the literal list 'name=[...]' of the constructor call."""
    return re.findall(r"'(\w+)'", re.search(name + r'=\[([^\]]*)\]', source).group(1))


@pytest.mark.parametrize('module_name, class_name', STATES.items())
def test_a_state_is_one_class_whose_docstring_says_its_interface(module_name, class_name):
    module = importlib.import_module(f'hand_eye_flexbe_states.{module_name}')
    source = inspect.getsource(module)
    state = getattr(module, class_name)
    assert re.findall(r'^class (\w+)\(EventState\):\n    """$', source, re.M) == [class_name]
    assert issubclass(state, EventState)
    arguments = list(inspect.signature(state.__init__).parameters)[1:]
    assert re.findall(r'^    -- (\w+) +\w+ +\S', state.__doc__, re.M) == arguments
    constructor = re.search(r'    def __init__\(self, ([^)]*)\):', source).group(1)
    assert '=' not in constructor  # every argument is given by the behavior: no hidden values
    assert re.findall(r'^    <= (\w+) +\S', state.__doc__, re.M) == listed(source, 'outcomes')
    for mark, keys in (('>#', 'input_keys'), ('#>', 'output_keys')):
        written = re.findall(rf'^    {mark} (\w+) +\w+ +\S', state.__doc__, re.M)
        assert written == (listed(source, keys) if f'{keys}=' in source else [])


@pytest.mark.parametrize('module_name', HELPERS)
def test_a_helper_is_not_taken_for_a_state(module_name):
    module = importlib.import_module(f'hand_eye_flexbe_states.{module_name}')
    assert '(EventState)' not in inspect.getsource(module)


def test_the_states_load_without_the_realsense_library():
    with pytest.raises(ImportError):
        importlib.import_module('pyrealsense2_is_not_needed_to_load_the_states')
    source = inspect.getsource(importlib.import_module('hand_eye_flexbe_states.camera_stream'))
    inside_a_method = ['        ']
    assert re.findall(r'^(\s*)import pyrealsense2', source, re.M) == inside_a_method
