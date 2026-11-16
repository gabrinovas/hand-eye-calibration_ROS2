# Copyright 2026 Gabriel Novas
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""The behaviors: what their manifests say, the layout FlexBE reads and what each one builds."""

import glob
import importlib
import inspect
import os
import re
import threading
import xml.etree.ElementTree as ElementTree

from ament_index_python.packages import get_package_prefix
import pytest
import rclpy
from rclpy.executors import MultiThreadedExecutor

PACKAGE = 'hand_eye_flexbe_behaviors'
MANIFESTS = sorted(glob.glob(os.path.join(get_package_prefix(PACKAGE), 'lib', PACKAGE, 'manifest',
                                          '**', '*.xml'), recursive=True))
TAGS = ('MANUAL_IMPORT', 'MANUAL_INIT', 'MANUAL_CREATE', 'MANUAL_FUNC')
UR_MOVEIT = ['ros2', 'launch', 'ur_moveit_config', 'ur_moveit.launch.py', 'ur_type:=ur5e']
CAPTURE = ('capture_and_calibrate_sm', 'CaptureAndCalibrateSM')
RECOMPUTE = ('test_detection_only_sm', 'TestDetectionOnlySM')
CAMERA = 'calibrations/realsense_d435i'


def behavior_of(manifest):
    """Return (manifest root, behavior class, source of its module) of a manifest file."""
    root = ElementTree.parse(manifest).getroot()
    executable = root.find('executable')
    module = importlib.import_module(executable.get('package_path'))
    return root, getattr(module, executable.get('class')), inspect.getsource(module)


@pytest.fixture(scope='module')
def node():
    rclpy.init()
    node = rclpy.create_node('behaviors_test')
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()
    yield node
    rclpy.shutdown()


@pytest.fixture
def build(node, tmp_path, monkeypatch):
    """Return the function that builds a behavior with parameter values and gives its states."""
    monkeypatch.setenv('HOME', str(tmp_path))  # the calibrations go to a temporary folder
    addresses = tmp_path / 'robot_addresses.yaml'
    addresses.write_text('default:\n  right: 192.0.2.101\n  kth_table: 192.0.2.194\n')
    monkeypatch.setenv('UR_ONROBOT_ROBOT_ADDRESSES', str(addresses))

    def make(module_name, class_name, **values):
        module = importlib.import_module(f'{PACKAGE}.{module_name}')
        behavior = getattr(module, class_name)(node)
        for name, value in values.items():
            setattr(behavior, name, value)
        return behavior.create()
    return make


def test_there_are_three_behaviors():
    assert [os.path.basename(path) for path in MANIFESTS] == [
        'camera_calibration.xml', 'capture_and_calibrate.xml', 'test_detection_only.xml']


@pytest.mark.parametrize('manifest', MANIFESTS)
def test_manifest_and_code_agree_and_flexbe_can_read_the_code(manifest):
    root, behavior, source = behavior_of(manifest)
    in_code = dict(re.findall(r"self\.add_parameter\('(\w+)', (.+)\)", source))
    in_manifest = {param.get('name'): param.get('default') for param in root.find('params')}
    assert {name: str(eval(value)) for name, value in in_code.items()} == in_manifest  # noqa: S307
    name = behavior.__name__
    for needed in (rf'^class {name}\(Behavior\):$', r'^    def __init__\(self, node\):$',
                   rf'^        super\({name}, self\)\.__init__\(\)$', r'^    def create\(self\):$',
                   rf"^        self\.name = '{re.escape(root.get('name'))}'$",
                   r'^        return _state_machine$'):
        assert re.search(needed, source, re.M), needed
    outside = source
    for tag in TAGS:  # the code of this repository is inside the tags FlexBE keeps
        assert source.count(f'# [{tag}]') == source.count(f'# [/{tag}]') == 1
        outside = re.sub(rf'# \[{tag}\].*?# \[/{tag}\]', '', outside, flags=re.S)
    assert 'charuco_calibrator' not in outside.split('class ')[1]
    assert not re.search(r'^from charuco_calibrator', outside, re.M)
    assert 'other' not in [option.get('value') for option in root.iter('option')]


def test_the_intrinsic_calibration_works_in_the_folder_of_the_camera(build, tmp_path):
    machine = build('camera_calibration_sm', 'camera_calibrationSM', pic_num=12)
    capture, calibration = machine['take_camera_cali_pic'], machine['camera_calibration']
    folder = f'{tmp_path}/{CAMERA}/intrinsic_calibrations/camera_calib_pictures'
    assert (capture.folder, capture.total, calibration.folder) == (folder, 12, folder)
    assert os.path.isdir(folder) and not os.path.exists('~')


def test_the_calibration_of_a_cell_uses_its_robot_its_folders_and_its_address(build, tmp_path):
    machine = build(*CAPTURE, cell='right')
    launch, capture = machine['Launch_MoveIt'], machine['Capture_Poses']
    detect, compute = machine['Process_Offline'], machine['Compute_Calibration']
    root = f'{tmp_path}/{CAMERA}/extrinsic_calibration/ur5e/right'
    assert launch.command == UR_MOVEIT + ['use_fake_hardware:=false', 'launch_rviz:=true',
                                          'robot_ip:=192.0.2.101']
    assert (capture.base_frame, capture.tool_frame) == ('base_link', 'tool0')
    assert capture.folders == {'pictures': f'{root}/pictures',
                               'robot_poses': f'{root}/robot_poses'}
    assert f'output_folder:={root}/charuco_table_poses' in detect.command
    assert 'camera_intrinsics_file:=latest' in detect.command
    assert 'base_frame:=base_link' in detect.command
    assert compute.folder == f'{root}/calibration_results'
    assert compute.about == {'robot_model': 'ur5e', 'cell': 'right', 'base_frame': 'base_link',
                             'camera_frame': 'camera_color_optical_frame', 'eye_in_hand': False}

    own = build(*CAPTURE, cell='right', robot_ip='192.0.2.77')
    assert own['Launch_MoveIt'].command[-1] == 'robot_ip:=192.0.2.77'
    fake = build(*CAPTURE, cell='left', use_fake_hardware=True)
    assert fake['Launch_MoveIt'].command == UR_MOVEIT + ['use_fake_hardware:=true',
                                                         'launch_rviz:=true']
    lite = build(*CAPTURE, cell='kth_table', robot_name='ufactory_lite6')
    assert lite['Launch_MoveIt'].command == [
        'ros2', 'launch', 'xarm_moveit_config', 'lite6_moveit_realmove.launch.py',
        'add_gripper:=true', 'launch_rviz:=true', 'robot_ip:=192.0.2.194']
    assert lite['Capture_Poses'].base_frame == 'link_base'
    results = lite['Compute_Calibration'].folder
    assert results.endswith('/ufactory_lite6/kth_table/calibration_results')


@pytest.mark.parametrize('values, error', [
    ({}, 'cell of the calibration is not valid'),
    ({'cell': 'left'}, "no address for the cell 'left'"),
    ({'cell': 'right', 'robot_name': 'abb'}, "Unknown robot 'abb'"),
    ({'cell': 'right', 'camera_type': 'other'}, "Unknown camera 'other'")])
def test_a_calibration_that_cannot_be_stored_or_reached_fails_at_start(build, values, error):
    with pytest.raises(Exception, match=rf'\[Fail-Fast\].*{error}'):
        build(*CAPTURE, **values)


def test_without_the_addresses_file_the_address_must_be_given(build, monkeypatch):
    monkeypatch.delenv('UR_ONROBOT_ROBOT_ADDRESSES')
    with pytest.raises(ValueError, match=r'\[Fail-Fast\] The address of the robot is not known'):
        build(*CAPTURE, cell='right')
    given = build(*CAPTURE, cell='right', robot_ip='192.0.2.5')
    assert given['Launch_MoveIt'].command[-1] == 'robot_ip:=192.0.2.5'


def test_the_stored_calibration_of_a_cell_is_computed_again_from_its_folders(build, tmp_path):
    machine = build(*RECOMPUTE, cell='right', eye_in_hand=True)
    detect, compute = machine['Process_Offline'], machine['Compute_Calibration']
    root = f'{tmp_path}/{CAMERA}/extrinsic_calibration/ur5e/right'
    assert f'pictures_folder:={root}/pictures' in detect.command
    assert 'eye_in_hand:=true' in detect.command
    assert 'camera_intrinsics_file:=latest' in detect.command
    assert compute.folder == f'{root}/calibration_results'
    assert compute.about['eye_in_hand'] is True
    with pytest.raises(ValueError, match=r'\[Fail-Fast\] The cell of the calibration is not'):
        build(*RECOMPUTE)
