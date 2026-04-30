#!/usr/bin/env python3
"""
FlexBE state to compute hand-eye calibration with VISP.
Launches VISP automatically if it is not running.
Adapted for UR5e.
"""

import configparser
import yaml
import os
import subprocess
import time
import psutil
import numpy as np
import signal
from geometry_msgs.msg import Transform
from visp_hand2eye_calibration.msg import TransformArray
from visp_hand2eye_calibration.srv import ComputeEffectorCameraQuick
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyServiceCaller
import tf_transformations

class ComputeCalibState(EventState):
    """
    Computes hand-eye calibration with VISP (auto-launched).
    
    -- eye_in_hand_mode      bool     True: eye-in-hand
    -- calibration_file_name string   Output file name
    -- customize_file        bool     Use custom name
    -- launch_visp           bool     Launch VISP automatically
    -- output_folder         string   Output folder
    
    ># base_h_tool           TransformArray  Robot poses (base→tool)
    ># camera_h_charuco      TransformArray  Board poses (camera→charuco)
    
    <= finish                Calibration completed
    <= failed                Error
    """
    
    def __init__(self, eye_in_hand_mode, calibration_file_name, 
                 customize_file=False, launch_visp=True, output_folder=None):
        super().__init__(
            outcomes=['finish', 'failed'],
            input_keys=['base_h_tool', 'camera_h_charuco']
        )
        
        self.eye_in_hand_mode = eye_in_hand_mode
        self.launch_visp = launch_visp
        self.visp_process = None
        self.calib_client = None
        self._service_ready = False
        
        # Configure output folders
        self.output_folder = output_folder or '/home/drims/calibrations'
        self.calib_results_folder = os.path.join(self.output_folder, 'calibration_results')
        os.makedirs(self.calib_results_folder, exist_ok=True)
        
        # Output file
        if customize_file:
            # If it already has an extension, remove it to handle consistency
            base_name = str(calibration_file_name)
            if base_name.endswith('.ini') or base_name.endswith('.yaml'):
                base_name = os.path.splitext(base_name)[0]
            self.calibration_base_name = base_name
        else:
            if eye_in_hand_mode:
                self.calibration_base_name = "eye_in_hand_calibration_ur5e"
            else:
                self.calibration_base_name = "eye_to_hand_calibration_ur5e"
        
        # Output files with correct extensions
        self.ini_file = os.path.join(self.calib_results_folder, f"{self.calibration_base_name}.ini")
        self.yaml_file = os.path.join(self.calib_results_folder, f"{self.calibration_base_name}.yaml")
        
        # Main output file
        self.main_output_file = '/home/drims/calibrations/camera_extrinsics.yaml'
        
        # Duplicate detections file to delete
        self.temp_detections_file = '/home/drims/calibrations/charuco_detections.yaml'
        
        # Config parser for INI
        self.config = configparser.ConfigParser()
        self.config.optionxform = str
        
        Logger.loginfo("="*60)
        Logger.loginfo("🔧 STATE: ComputeCalibState (UR5e)")
        Logger.loginfo("="*60)
        Logger.loginfo(f"🎯 Mode: {'Eye-in-hand' if eye_in_hand_mode else 'Eye-to-hand'}")
        Logger.loginfo(f"📁 Base file: {self.calibration_base_name}")
        Logger.loginfo(f"📂 Folder: {self.calib_results_folder}")
        Logger.loginfo(f"📄 INI file: {self.ini_file}")
        Logger.loginfo(f"📄 YAML file: {self.yaml_file}")
        Logger.loginfo(f"📄 Main file: {self.main_output_file}")
        Logger.loginfo(f"📄 Temp file to delete: {self.temp_detections_file}")
        Logger.loginfo(f"🚀 Auto-VISP: {launch_visp}")
    
    def on_start(self):
        """Initialize: launch VISP if necessary"""
        # Initialize proxy with FlexBE node
        ProxyServiceCaller.initialize(ComputeCalibState._node)
        
        if self.launch_visp:
            self._ensure_visp_running()
        
        # Now create the client
        self.calib_client = ProxyServiceCaller({
            '/compute_effector_camera_quick': ComputeEffectorCameraQuick
        })
        
        # Verify service is available
        if self.calib_client.is_available('/compute_effector_camera_quick'):
            self._service_ready = True
            Logger.loginfo("✅ VISP service available")
        else:
            Logger.logwarn("⏳ Waiting for VISP service...")
    
    def _ensure_visp_running(self):
        """Launches VISP if it is not running"""
        if self._is_visp_running():
            Logger.loginfo("✅ VISP is already running")
            return True
        
        Logger.loginfo("🚀 Launching VISP node...")
        
        try:
            # Convert boolean to string 'true'/'false' for ROS2
            eye_in_hand_str = 'true' if self.eye_in_hand_mode else 'false'
            
            cmd = [
                'ros2', 'run', 
                'visp_hand2eye_calibration', 
                'visp_hand2eye_calibration_calibrator',
                '--ros-args',
                '-p', f'eye_in_hand:={eye_in_hand_str}',
                '-p', 'camera_frame:=camera_color_optical_frame',
                '-p', 'marker_frame:=charuco_frame'
            ]
            
            Logger.loginfo(f"📋 Command: {' '.join(cmd)}")
            
            self.visp_process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                start_new_session=True,
                text=True
            )
            
            # Wait for it to start
            Logger.loginfo("⏳ Waiting for VISP to start...")
            
            for i in range(15):
                time.sleep(1)
                if self._is_visp_running():
                    Logger.loginfo(f"✅ VISP started successfully after {i+1}s")
                    return True
                
                # Check if process failed
                if self.visp_process.poll() is not None:
                    stdout, stderr = self.visp_process.communicate()
                    Logger.logerr(f"❌ VISP terminated prematurely")
                    if stderr:
                        Logger.logerr(f"Error: {stderr}")
                    return False
            
            Logger.logwarn("⚠️ VISP is not responding but might be starting...")
            return True
            
        except Exception as e:
            Logger.logerr(f"❌ Error launching VISP: {e}")
            return False
    
    def _is_visp_running(self):
        """Verifies if VISP is running"""
        try:
            # Search by process
            for proc in psutil.process_iter(['pid', 'cmdline']):
                try:
                    if proc.info['cmdline'] and any('visp_hand2eye_calibration_calibrator' in cmd for cmd in proc.info['cmdline'] if cmd):
                        return True
                except:
                    pass
            
            # Also check service
            try:
                result = subprocess.run(
                    ['ros2', 'service', 'list'],
                    capture_output=True,
                    text=True,
                    timeout=2
                )
                if '/compute_effector_camera_quick' in result.stdout:
                    return True
            except:
                pass
                
        except Exception as e:
            Logger.logwarn(f"⚠️ Error checking VISP: {e}")
        
        return False
    
    def execute(self, userdata):
        """Executes the calibration with the received data"""
        
        # Verify service is ready
        if not self._service_ready:
            if self.calib_client and self.calib_client.is_available('/compute_effector_camera_quick'):
                self._service_ready = True
                Logger.loginfo("✅ VISP service is now available")
            else:
                Logger.logwarn("⏳ Waiting for VISP service to be available...")
                return None  # Do not fail, just wait
        
        # Verify data
        if not hasattr(userdata.base_h_tool, 'transforms') or len(userdata.base_h_tool.transforms) == 0:
            Logger.logerr("❌ No robot poses received")
            return 'failed'
        
        if not hasattr(userdata.camera_h_charuco, 'transforms') or len(userdata.camera_h_charuco.transforms) == 0:
            Logger.logerr("❌ No board poses received")
            return 'failed'
        
        # Verify pose count matches
        if len(userdata.base_h_tool.transforms) != len(userdata.camera_h_charuco.transforms):
            Logger.logerr(f"❌ Pose count mismatch: robot={len(userdata.base_h_tool.transforms)}, board={len(userdata.camera_h_charuco.transforms)}")
            return 'failed'
        
        num_poses = len(userdata.base_h_tool.transforms)
        Logger.loginfo(f"📊 Calibrating with {num_poses} poses")
        
        # Verify service available
        if not self.calib_client.is_available('/compute_effector_camera_quick'):
            Logger.logerr("❌ VISP service not available")
            return 'failed'
        
        # Prepare request
        req = ComputeEffectorCameraQuick.Request()
        req.camera_object = TransformArray()
        req.world_effector = TransformArray()
        req.camera_object.header = userdata.camera_h_charuco.header
        req.world_effector.header = userdata.base_h_tool.header
        
        # Process based on mode
        if self.eye_in_hand_mode:
            # Eye-in-hand mode: use directly
            Logger.loginfo("🔄 Eye-in-hand mode: using poses directly")
            req.world_effector.transforms = userdata.base_h_tool.transforms
            req.camera_object.transforms = userdata.camera_h_charuco.transforms
        else:
            # Eye-to-hand mode: invert only the robot transform
            Logger.loginfo("🔄 Eye-to-hand mode: inverting only robot transform (base_h_tool)")
            
            for t in userdata.base_h_tool.transforms:
                inv = self._invert_transform(t)
                req.world_effector.transforms.append(inv)
            
            # Camera to charuco transform is NOT inverted in eye-to-hand
            # The equation is: (bMe)^-1 * bMc * cMo = eMo (Constant)
            for t in userdata.camera_h_charuco.transforms:
                req.camera_object.transforms.append(t)
        
        try:
            # Call service
            Logger.loginfo("📡 Calling VISP calibration service...")
            res = self.calib_client.call('/compute_effector_camera_quick', req)
            
            if res is None:
                Logger.logerr("❌ Service did not return a response")
                return 'failed'
            
            # Show result
            Logger.loginfo("="*60)
            Logger.loginfo("✅ CALIBRATION COMPLETED - UR5e")
            Logger.loginfo("="*60)
            Logger.loginfo(f"📐 Translation (meters):")
            Logger.loginfo(f"   x = {res.effector_camera.translation.x:.6f}")
            Logger.loginfo(f"   y = {res.effector_camera.translation.y:.6f}")
            Logger.loginfo(f"   z = {res.effector_camera.translation.z:.6f}")
            Logger.loginfo(f"🌀 Quaternion:")
            Logger.loginfo(f"   qx = {res.effector_camera.rotation.x:.6f}")
            Logger.loginfo(f"   qy = {res.effector_camera.rotation.y:.6f}")
            Logger.loginfo(f"   qz = {res.effector_camera.rotation.z:.6f}")
            Logger.loginfo(f"   qw = {res.effector_camera.rotation.w:.6f}")
            
            # Convert to Euler angles for UR
            quat = [res.effector_camera.rotation.x,
                   res.effector_camera.rotation.y,
                   res.effector_camera.rotation.z,
                   res.effector_camera.rotation.w]
            euler = tf_transformations.euler_from_quaternion(quat)
            Logger.loginfo(f"📐 Euler angles (rad):")
            Logger.loginfo(f"   roll = {euler[0]:.6f}, pitch = {euler[1]:.6f}, yaw = {euler[2]:.6f}")
            
            # Save result
            self._save_calibration(res.effector_camera, num_poses)
            
            # Remove temporary detections file
            self._cleanup_temp_files()
            
            return 'finish'
            
        except Exception as e:
            Logger.logerr(f"❌ Calibration error: {e}")
            return 'failed'
    
    def _invert_transform(self, t):
        """Inverts a transformation"""
        # Homogeneous matrix
        T = np.eye(4)
        quat = [t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w]
        R = tf_transformations.quaternion_matrix(quat)[:3, :3]
        T[:3, :3] = R
        T[:3, 3] = [t.translation.x, t.translation.y, t.translation.z]
        
        # Invert
        T_inv = np.linalg.inv(T)
        
        # Create inverse transform
        inv = Transform()
        inv.translation.x = float(T_inv[0, 3])
        inv.translation.y = float(T_inv[1, 3])
        inv.translation.z = float(T_inv[2, 3])
        
        quat_inv = tf_transformations.quaternion_from_matrix(T_inv)
        inv.rotation.x = float(quat_inv[0])
        inv.rotation.y = float(quat_inv[1])
        inv.rotation.z = float(quat_inv[2])
        inv.rotation.w = float(quat_inv[3])
        
        return inv
    
    def _cleanup_temp_files(self):
        """Deletes temporary files after calibration"""
        try:
            if os.path.exists(self.temp_detections_file):
                os.remove(self.temp_detections_file)
                Logger.loginfo(f"🧹 Temporary file removed: {self.temp_detections_file}")
            else:
                Logger.loginfo(f"📂 Temporary file not found (already removed): {self.temp_detections_file}")
        except Exception as e:
            Logger.logwarn(f"⚠️ Could not delete temporary file {self.temp_detections_file}: {e}")
    
    def _save_calibration(self, transform, num_poses):
        """Saves the calibration in INI, YAML format and main file"""
        
        # ===== Save INI =====
        ini_path = self.ini_file
        
        if os.path.exists(ini_path):
            self.config.read(ini_path)
        
        if not self.config.has_section("hand_eye_calibration"):
            self.config.add_section("hand_eye_calibration")
        
        self.config.set("hand_eye_calibration", "x", str(transform.translation.x))
        self.config.set("hand_eye_calibration", "y", str(transform.translation.y))
        self.config.set("hand_eye_calibration", "z", str(transform.translation.z))
        self.config.set("hand_eye_calibration", "qx", str(transform.rotation.x))
        self.config.set("hand_eye_calibration", "qy", str(transform.rotation.y))
        self.config.set("hand_eye_calibration", "qz", str(transform.rotation.z))
        self.config.set("hand_eye_calibration", "qw", str(transform.rotation.w))
        self.config.set("hand_eye_calibration", "num_poses_used", str(num_poses))
        self.config.set("hand_eye_calibration", "eye_in_hand", str(self.eye_in_hand_mode))
        self.config.set("hand_eye_calibration", "robot_model", "ur5e")
        self.config.set("hand_eye_calibration", "timestamp", str(time.time()))
        
        with open(ini_path, 'w') as f:
            self.config.write(f)
        
        Logger.loginfo(f"💾 Calibration saved (INI): {ini_path}")
        
        # ===== Save YAML =====
        yaml_path = self.yaml_file
        
        # Complete transformation matrix
        T = np.eye(4)
        quat = [transform.rotation.x, transform.rotation.y, 
                transform.rotation.z, transform.rotation.w]
        R = tf_transformations.quaternion_matrix(quat)[:3, :3]
        T[:3, :3] = R
        T[:3, 3] = [transform.translation.x, transform.translation.y, transform.translation.z]
        
        # Convert to Euler angles for UR
        euler = tf_transformations.euler_from_quaternion(quat)
        
        # Convert matrix to native float list
        transform_matrix = []
        for row in T.tolist():
            transform_matrix.append([float(x) for x in row])
        
        # Convert all numpy values to native floats
        calib_data = {
            'calibration_date': time.strftime("%Y-%m-%d %H:%M:%S"),
            'robot_model': 'ur5e',
            'eye_in_hand': bool(self.eye_in_hand_mode),
            'num_poses_used': int(num_poses),
            'transform_matrix': transform_matrix,
            'translation': {
                'x': float(transform.translation.x),
                'y': float(transform.translation.y),
                'z': float(transform.translation.z)
            },
            'rotation_quaternion': {
                'x': float(transform.rotation.x),
                'y': float(transform.rotation.y),
                'z': float(transform.rotation.z),
                'w': float(transform.rotation.w)
            },
            'rotation_euler_rad': {
                'roll': float(euler[0]),
                'pitch': float(euler[1]),
                'yaw': float(euler[2])
            },
            'rotation_euler_deg': {
                'roll': float(np.degrees(euler[0])),
                'pitch': float(np.degrees(euler[1])),
                'yaw': float(np.degrees(euler[2]))
            }
        }
        
        with open(yaml_path, 'w') as f:
            yaml.dump(calib_data, f, default_flow_style=False, sort_keys=False, indent=2)
        
        Logger.loginfo(f"💾 Calibration saved (YAML): {yaml_path}")
        
        # ===== Save main file =====
        with open(self.main_output_file, 'w') as f:
            yaml.dump(calib_data, f, default_flow_style=False, sort_keys=False, indent=2)
        
        Logger.loginfo(f"💾 Calibration saved in main file: {self.main_output_file}")
        
        # ===== Save extrinsic_matrix.yaml with specific format =====
        extrinsic_file = os.path.join(self.output_folder, 'extrinsic_matrix.yaml')
        
        # T is the transformation matrix returned by ViSP (eMc or bMc).
        # Transforms points from the camera frame to the parent frame (effector or base).
        # Therefore, T is mathematically T_c2w. The inverse is T_w2c.
        T_c2w = T
        T_w2c = np.linalg.inv(T)
        
        # Convert matrices to flat lists of 16 elements
        T_w2c_flat = T_w2c.flatten().tolist()
        T_c2w_flat = T_c2w.flatten().tolist()
        
        # Convert all values to native float with high precision
        T_w2c_flat = [float(x) for x in T_w2c_flat]
        T_c2w_flat = [float(x) for x in T_c2w_flat]
        
        # Create structure with 'camera' key and index 1
        extrinsic_data = {
            'camera': {
                1: {
                    'T_w2c': T_w2c_flat,
                    'T_c2w': T_c2w_flat
                }
            }
        }
        
        with open(extrinsic_file, 'w') as f:
            # Use default_flow_style=None for readable but compact format
            yaml.dump(extrinsic_data, f, default_flow_style=None, sort_keys=False, indent=2)
        
        Logger.loginfo(f"💾 Extrinsic matrix saved in: {extrinsic_file}")
    
    def on_stop(self):
        """Clean up VISP process on finish"""
        if self.visp_process:
            Logger.loginfo("🛑 Stopping VISP...")
            try:
                os.killpg(os.getpgid(self.visp_process.pid), signal.SIGINT)
                self.visp_process.wait(timeout=5)
            except:
                try:
                    self.visp_process.terminate()
                    self.visp_process.wait(timeout=3)
                except:
                    try:
                        self.visp_process.kill()
                    except:
                        pass
            self.visp_process = None
            Logger.loginfo("✅ VISP stopped")
