# Copyright 2026 Gabriel Novas
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Helpers of the states, without a camera, a robot or a display."""

import sys
import time

from charuco_calibrator.config_loader import ConfigError
import cv2
from hand_eye_flexbe_states import camera_stream, capture_window, managed_process
from hand_eye_flexbe_states.camera_stream import CameraStream
from hand_eye_flexbe_states.capture_window import CANCEL, CAPTURE, CaptureWindow
from hand_eye_flexbe_states.managed_process import executable_running, ManagedProcess
from hand_eye_flexbe_states.settings import capture_ui, processes
import numpy as np
import pytest


def test_the_configuration_of_the_states_is_complete():
    assert processes('visp', 'service') == '/compute_effector_camera_quick'
    assert processes('detector', 'detection_timeout_s') == 120
    assert capture_ui('keys') == {'capture': [13, 32], 'cancel': [27]}
    for section in ('intrinsic', 'hand_eye'):
        look = capture_ui(section)
        assert len(look['lines']) == 4 and look['window']['width'] == 1280
    assert capture_ui('intrinsic', 'warm_up_s') == 1.0
    assert capture_ui('hand_eye', 'warm_up_s') == 0.0
    with pytest.raises(ConfigError, match=r"\[Fail-Fast\] 'visp.no_such_value' is missing"):
        processes('visp', 'no_such_value')


def test_a_program_is_started_seen_running_and_stopped():
    marker = 'calibration_test_program'
    program = ManagedProcess([sys.executable, '-c', f'import time; time.sleep(60)  # {marker}'],
                             quiet=False)
    assert not program.started() and not program.ended() and not executable_running(marker)
    program.start()
    assert program.started() and not program.ended() and executable_running(marker)
    began = time.time()
    program.stop()
    assert time.time() - began < processes('stop', 'interrupt_wait_s')
    assert not program.started() and not executable_running(marker)
    quick = ManagedProcess([sys.executable, '-c', 'pass'], quiet=True)
    quick.start()
    quick.process.wait()
    assert quick.ended()
    quick.stop()  # stopping what already ended is not an error
    assert not quick.started()


def test_a_program_opens_its_windows_on_the_display_of_who_starts_it(tmp_path, monkeypatch):
    seen = tmp_path / 'display.txt'
    monkeypatch.setenv('DISPLAY', ':7')
    show = f"import os; open(r'{seen}', 'w').write(os.environ['DISPLAY'])"
    program = ManagedProcess([sys.executable, '-c', show], quiet=True)
    program.start()
    program.process.wait()
    assert seen.read_text() == ':7'  # the display is inherited, never replaced
    assert 'environment' not in processes('moveit')


def test_a_ros_listing_that_cannot_be_asked_counts_as_not_listed_and_says_so(monkeypatch):
    def unavailable(*_args, **_kwargs):
        raise OSError('ros2 is not here')
    said = []
    monkeypatch.setattr(managed_process.subprocess, 'run', unavailable)
    monkeypatch.setattr(managed_process.Logger, 'logwarn', said.append)
    assert managed_process.ros_lists('node', ['move_group']) is False
    assert len(said) == 1  # not silent
    assert 'ros2 is not here' in said[0] and 'move_group' in said[0]


class Pipeline:
    """Stand-in for a RealSense pipeline that only starts with the streams it accepts."""

    accepted, started, devices, stopped = (), [], [], 0

    def start(self, config):
        if config.stream[:2] not in Pipeline.accepted:
            raise RuntimeError(f'cannot resolve {config.stream[:2]}')
        Pipeline.started.append(config.stream)
        Pipeline.devices.append(config.device)

    def stop(self):
        Pipeline.stopped += 1


class Config:
    def enable_device(self, serial):
        self.device = serial

    def enable_stream(self, _kind, width, height, _pixel_format, fps):
        self.stream = (width, height, fps)


@pytest.fixture
def realsense(monkeypatch):
    """Return the Pipeline class of a pyrealsense2 module made of the stand-ins above."""
    module = type(sys)('pyrealsense2')
    module.pipeline, module.config = Pipeline, Config
    module.stream = module.format = type('names', (), {'color': 'color', 'bgr8': 'bgr8'})
    monkeypatch.setitem(sys.modules, 'pyrealsense2', module)
    monkeypatch.setattr(Pipeline, 'started', [])
    monkeypatch.setattr(Pipeline, 'devices', [])
    monkeypatch.setattr(Pipeline, 'stopped', 0)
    return Pipeline


def test_the_camera_starts_with_its_one_stream_or_not_at_all(realsense):
    realsense.accepted = ((1920, 1080), (1280, 720))
    camera = CameraStream('realsense_d435i', '42')
    assert camera.start(warm_up_s=0.0) == 'RealSense 42 1920x1080 at 30 Hz'
    assert realsense.started == [(1920, 1080, 30)]
    assert realsense.devices == ['42']  # the unit asked for, never any camera connected
    camera.stop()
    assert realsense.stopped == 1 and camera.pipeline is None
    realsense.accepted = ((1280, 720),)  # a camera that only gives another resolution
    with pytest.raises(RuntimeError, match=r'\[Fail-Fast\].*1920x1080 at 30 Hz.*cannot resolve'):
        CameraStream('realsense_d435i', '42').start(warm_up_s=0.0)
    assert realsense.started == [(1920, 1080, 30)]  # no other resolution was tried
    assert set(camera.settings) == {'driver', 'color', 'frame_timeout_ms'}
    with pytest.raises(ConfigError, match=r"\[Fail-Fast\] Unknown camera 'other'"):
        CameraStream('other', '42')
    with pytest.raises(RuntimeError, match='has not been started'):
        CameraStream('usb', 'usb0').picture()
    assert camera_stream.REALSENSE == 'realsense'


def test_the_window_shows_the_texts_and_tells_the_key(monkeypatch):
    drawn, keys = [], [32, 27, 255, 13]
    monkeypatch.setattr(cv2, 'namedWindow', lambda *args: drawn.append(('window', args[0])))
    monkeypatch.setattr(cv2, 'resizeWindow', lambda *args: drawn.append(('size', args[1:])))
    monkeypatch.setattr(cv2, 'imshow', lambda title, image: drawn.append(('shown', image.shape)))
    monkeypatch.setattr(cv2, 'waitKey', lambda _wait: keys.pop(0) if keys else 255)
    monkeypatch.setattr(cv2, 'destroyAllWindows', lambda: drawn.append(('closed',)))
    texts = []
    put_text = cv2.putText
    monkeypatch.setattr(cv2, 'putText', lambda image, text, *rest: texts.append(text)
                        or put_text(image, text, *rest))
    window = CaptureWindow('hand_eye', robot='UR5E')
    picture = np.full((1080, 1920, 3), 200, dtype=np.uint8)
    asked = [window.show(picture, taken=3, total=20, remaining=17) for _ in range(4)]
    assert asked == [CAPTURE, CANCEL, None, CAPTURE]
    assert picture.min() == 200  # the picture itself is not drawn on
    assert drawn[:2] == [('window', 'UR5E CALIBRATION - Pose capture'), ('size', (1280, 720))]
    assert texts[:4] == ['UR5E - EXTRINSIC CALIBRATION', 'CAPTURED: 3/20', 'REMAINING: 17',
                         'SPACE: Capture | ESC: Cancel']
    assert all(text.isascii() for text in texts)  # OpenCV draws nothing else
    window.close()
    assert drawn[-1] == ('closed',) and window.opened is False
    with pytest.raises(ConfigError, match=r"\[Fail-Fast\] The text 'Saving in: \{folder\}'"):
        CaptureWindow('intrinsic').show(picture, taken=0, total=5, remaining=5)
    assert capture_window.KEY_MASK == 0xFF
