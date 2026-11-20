# Universal Automatical Hand-Eye-Calibration

This hand-eye calibration framework integrates [FlexBE](http://philserver.bplaced.net/fbe/download.php) and [MoveIt!2](https://moveit.ros.org/) to make the calibration process intuitive for beginners, while allowing fast deployment on different robotic platforms.

## Features
- **Dual Calibration Modes:** Mathematically robust and fully validated support for both **Eye-in-hand** (camera mounted on the robot's end-effector) and **Eye-to-hand** (fixed camera in the environment, target mounted on the robot) setups.
- **FlexBE GUI Integration:** Manage the capture, detection, and calculation processes effortlessly through visual state machines.
- **High-Precision Charuco Detection:** Powered by OpenCV, using Charuco boards to maximize camera pose estimation accuracy.
- **Standardized ROS2 Architecture:** Connects smoothly with MoveIt!2 and the ViSP hand-eye calibration solver (`vision_visp`).

## How It Works (Architecture Overview)

The calibration pipeline automates the complex geometric relationship mapping between the robot and the camera through a streamlined process:

### 1. Data Collection & Robot Kinematics (`hand_eye_flexbe_behaviors`)
Using MoveIt!2, FlexBE commands the robot arm to move through a predefined set of waypoint poses. For each waypoint, the system records the exact spatial transformation from the robot's **Base** to its **End-Effector** ($^{b}M_{e}$). This ensures a robust variation in viewpoints, which is mathematically essential for the solver to converge optimally.

### 2. High-Fidelity Target Detection (`charuco_calibrator`)
At every waypoint, the camera captures an image. These images are processed by our OpenCV-based Charuco detector. Unlike standard ArUco markers, Charuco boards use internal chessboard corners to achieve sub-pixel accuracy. The detector extracts the transformation matrix from the **Camera** to the **Target Board** ($^{c}M_{o}$).

### 3. Kinematic Solver (`vision_visp`)
Once all poses and detections are gathered, the data is fed into the ViSP (Visual Servoing Platform) `visp_hand2eye_calibration` solver. The solver parses the classical Hand-Eye mathematical formulation $AX = XB$:
- **Eye-in-Hand Mode:** The camera moves with the arm, solving for the static transformation between the End-Effector and the Camera ($^{e}M_{c}$).
- **Eye-to-Hand Mode:** The camera is fixed and observes the board mounted on the moving arm. The system automatically handles the strict mathematical inversion of the robot's kinematics to solve for the transformation between the Base and the Camera ($^{b}M_{c}$).

### 4. Output Generation
Finally, the computed $4 \times 4$ matrix, its inverse and the quaternion are written to one `.yaml` file per calibration, together with what it is the calibration of (robot, cell and frames). That file can be injected directly into subsequent robotic grasping, perception, or visual-servoing pipelines without any further mathematical conversion.

## Acknowledgment
This charuco marker detection is forked from the [charuco_detector](https://github.com/carlosmccosta/charuco_detector).
And edit it to ROS2 version.

## Requirements 
This package requires a system setup with ROS. It is recommended to use **Ubuntu 22.04 with ROS
humble**.

camera SDK and ROS2 package is needed. Here we use [Linux Distribution](https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md#installing-the-packages) and [Realsense-ros](https://github.com/IntelRealSense/realsense-ros/tree/ros2-development)


To make sure that program isn't affected by python version, it is highly recommended to use a docker, 
we have build a environment that all Requirement for this package is build, 
See the [ubuntu-docker](https://github.com/errrr0501/ubuntu_docker) on information how to set this up.

## Building

```bash

# Install MoveIt 
sudo apt install ros-$ROS_DISTRO--moveit

# Clone this hand-eye-cliabtion package, flexbe_engine, flexbe_app, charuco_detector to your workspace
git clone --recursive https://github.com/tku-iarc/hand-eye-calibration_ROS2.git

# build the workspace
colcon build --symlink-install

# activate the workspace (ie: source it)
source install/setup.bash
```
## Alternative: Robot arm package
Note: MoveIt has been used on over 126 robots by the community, in this automatical hand-eye-calibration method, 
we chose it to do robot control and motion planning, therefore, user needs to use their own robot and it's MoveIt package.
For example, we use [Universal Robot 5 e-Series](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/tree/humble) as sample.

## Setting up a robot for hand-eye-calibration

### Prepare the camera also do camera calibration first
Note:In our method we use Realsense camera model and do camera calibration with charuco marker board.
No matter what kind of calibration method just make sure you have Distortion parameter and Intrinsic parameter,
like we put in [directory](https://github.com/errrr0501/charuco_detector_ROS2/tree/main/charuco_detector/config/camera_calibration).
### Camera calibration
we call Realsense with python, calibrate with charuco marker board, and everything is under FlexBE.
If you want to use, call FlexBE:

```bash
ros2 launch flexbe_app flexbe_full.launch
```

Then, press `Load Behavior` on the top, and select `camera_calibration` in left window.

After this, you can press `Runtime Control` on the top, execution window will show:

<center><img src="doc/resource/camera_calibration_execution.png" alt="camera_calibration_execution" style="width: 80%;"/></center>

Before pressing `Start Execution`, the parameter `pic_num` decides how many calibration pictures you will take and
`camera_type` which camera is calibrated. The board is not a parameter: its squares, sizes and dictionary are in
`charuco_calibrator/config/board.yaml`, the same board every calibration uses.

The pictures go to `~/calibrations/<camera>/intrinsic_calibrations/<serial>/camera_calib_pictures/` and the result to
`~/calibrations/<camera>/intrinsic_calibrations/<serial>/` (see below). `<serial>` is the serial number of the camera unit: the behavior parameter `camera_serial`, or the only RealSense connected when it is empty.

### Prepare the robot
Activate robot and open with MoveIt. 

As example using Universal Robot 5 e-Series:

- To start hardware, use:
```bash
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=<UR_TYPE> robot_ip:=<IP_OF_THE_ROBOT> launch_rviz:=false
```
- To do test with moveIt, use:
```bash
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5e launch_rviz:=true
```

As example using UFactory Lite 6:
- MoveIt package: `xarm_moveit_config` (or `ufactory_moveit_config`)
- Frames: `base_frame` = `link_base`, `tool_frame` = `link_eef`
- To test with MoveIt (fake/simulation):
```bash
ros2 launch xarm_moveit_config lite6_moveit_fake.launch.py add_gripper:=true launch_rviz:=true
```
- To test with real robot hardware:
```bash
ros2 launch xarm_moveit_config lite6_moveit_realmove.launch.py robot_ip:=<IP_OF_THE_ROBOT> add_gripper:=true launch_rviz:=true
```

## Multi-Camera & Multi-Robot Calibration Results Hierarchy

This repository supports dynamic path resolution per camera model (e.g., `realsense_d435i`) and robot platform (**Universal Robots UR5e** and **UFactory Lite 6**).

### Folder Organization (`~/calibrations/`)
- **Camera Intrinsics (`~/calibrations/<camera_model>/intrinsic_calibrations/<serial>/`)**:
  - `camera_intrinsics.yaml`: **the calibration in use**. It is what the applications and the camera-robot calibration read, and what `latest` means. One format, with everything known: camera matrix, distortion coefficients, resolution, reprojection error, date and board.
  - `camera_intrinsics_<YYYY-MM-DD_HH-MM-SS>.yaml`: a copy of each calibration, in the same format, kept as history. To go back to one of them, copy it over `camera_intrinsics.yaml`.
  - `camera_calib_pictures/`: the pictures of the last capture.
- **Extrinsic Calibrations per Camera, Robot & Cell (`~/calibrations/<camera_model>/extrinsic_calibration/<robot_name>/<cell>/`)**:
  - A calibration belongs to one camera, one robot model **and one cell** (the place where that robot and that camera stand): the same robot model in another cell has another camera pose.
  - Each cell folder holds `pictures/`, `robot_poses/`, `charuco_table_poses/` and `calibration_results/` (`camera_extrinsics.yaml`, the only result file). Example: `~/calibrations/realsense_d435i/extrinsic_calibration/ur5e/right/`.
  - `camera_extrinsics.yaml` says what it is the calibration of: `robot_model`, `cell`, `base_frame` and `camera_frame`, besides the transform. The programs that use it check those fields.
  - Folders, file names and frames are decided in one place: `charuco_calibrator/config/layout.yaml` and `robots.yaml` (read by `charuco_calibrator/charuco_calibrator/calibration_layout.py`).
- **Robot Configuration Files (`~/calibrations/robot_calibration/<robot_name>/`)**:
  - The factory kinematics calibration extracted from each robot.

### FlexBE Behavior Parameters (`capture_and_calibrate`)
- **`camera_type`**: A camera of `charuco_calibrator/config/cameras.yaml` (`realsense_d435i`). Its name is also the folder of its calibrations.
- **`robot_name`**: Set to `ur5e` or `ufactory_lite6`.
  - **UR5e**: Uses `ur_moveit_config`, `ur_moveit.launch.py`, `base_frame` = `base_link`, `tool_frame` = `tool0`. (`base_link`, not `base`: the UR `base` frame is the same point turned half a turn about the vertical axis, and the applications work in `base_link`.)
  - **UFactory Lite 6**: Uses `xarm_moveit_config`, `lite6_moveit_fake.launch.py` (sim) / `lite6_moveit_realmove.launch.py` (real), `base_frame` = `link_base`, `tool_frame` = `link_eef`.
- **`cell`**: The cell the robot and the camera stand in (for example `right`, `left`, `basic`, `kth_table`). Required: the calibration is stored and identified per cell.
- **`robot_ip`**: Leave it empty. The address of the robot of the cell is read from the robot addresses file of the installation, whose path is in the environment variable `UR_ONROBOT_ROBOT_ADDRESSES` (`default: {<cell>: <address>}`). No address is written in this repository. Type one only to use another robot.
- **`camera_intrinsics_file`**: `latest` to use the intrinsic calibration in use (`camera_intrinsics.yaml`), or the name of one of the dated copies of `~/calibrations/<camera_model>/intrinsic_calibrations/<serial>/`.
- **`use_fake_hardware`**: `True` for simulation (fake controllers), `False` for real hardware.

---

## Result Files
Results are exported per robot and cell under `~/calibrations/<camera_model>/extrinsic_calibration/<robot_name>/<cell>/calibration_results/`.
`camera_extrinsics.yaml` is the result, and the only file: robot, cell, frames, date, number of poses, the pose of
the camera as `transform_matrix` (camera to robot base), `inverse_transform_matrix` (robot base to camera),
translation, quaternion and Euler angles. `charuco_table_poses/` keeps what the result was computed from: `calibration_pairs.yaml` (robot pose and board pose of every picture used) and a copy of each picture with the
detection drawn.

## Configuration

No value is written in the code: every program reads the YAML files below, and a file or a value that is missing
stops it with a `[Fail-Fast]` error that names it.

| File | What it holds |
|---|---|
| `charuco_calibrator/config/board.yaml` | The ChArUco board: squares, sizes, dictionary |
| `charuco_calibrator/config/detection.yaml` | Markers and corners a picture needs, for each kind of calibration |
| `charuco_calibrator/config/layout.yaml` | Folders and file names of the calibrations, camera frame |
| `charuco_calibrator/config/robots.yaml` | Robots: calibration folder, frames, MoveIt launch; variable of the addresses file |
| `charuco_calibrator/config/cameras.yaml` | Cameras: driver and the one stream the pictures are taken with |
| `charuco_calibrator/config/hand_eye_node.yaml` | Topics of the node `charuco_hand_eye` |
| `hand_eye_flexbe_states/config/processes.yaml` | Programs the states start (ViSP, the detection node, MoveIt) and their waits |
| `hand_eye_flexbe_states/config/capture_ui.yaml` | Window of the captures: texts, keys, sizes |

Two things are not in those files because FlexBE needs them where they are: the parameters of each behavior and
their defaults are in its manifest (`hand_eye_flexbe_behaviors/manifest/`) and repeated in its `add_parameter`
lines (a test checks that they agree).

To add a robot, add its entry to `robots.yaml` and its option to the `robot_name` parameter of the manifests. To add
a camera, add its entry to `cameras.yaml` and its option to the `camera_type` parameter.

## The behaviors and the FlexBE editor

The three `*_sm.py` files keep the layout the FlexBE editor generates, because the FlexBE app reads their text to
show and run them. They are maintained by hand, with the code of this repository inside the `[MANUAL]` tags: use the
editor to run the behaviors, not to save them (a save writes the file again and keeps only what is inside the tags).
The states follow the same rule: one class per file, with the interface in its docstring.

## Tests and linters

Each of the three packages (`charuco_calibrator`, `hand_eye_flexbe_states`, `hand_eye_flexbe_behaviors`) has its
tests in `test/` and is checked with `ament_flake8`, `ament_pep257` and `ament_copyright`:

```bash
colcon build --packages-select charuco_calibrator hand_eye_flexbe_states hand_eye_flexbe_behaviors
source install/setup.bash
cd src/<package> && python3 -m pytest test && ament_flake8 . && ament_pep257 . && ament_copyright .
```

None of them needs a robot, a camera or a display. What they cannot cover is the capture itself (the camera, the
window and the pose of the real robot).
