#!/usr/bin/env python3
"""
FlexBE state to capture images and robot poses simultaneously.
Adapted for UR5e.
"""

from flexbe_core import EventState, Logger
import cv2
import pyrealsense2 as rs
import numpy as np
import os
import time
import glob
import yaml
from geometry_msgs.msg import TransformStamped
import rclpy
from rclpy.duration import Duration
import tf2_ros
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class TakePoseAndPictureState(EventState):
    """
    Captures images and robot poses simultaneously.
    
    -- total_poses       int     TOTAL number of poses to capture
    -- camera_type       str     'realsense' or 'usb'
    -- base_frame        str     Robot base frame (base_link)
    -- tool_frame        str     End-effector frame (tool0)
    -- pictures_folder   str     Folder for images
    -- robot_poses_folder str    Folder for poses
    -- output_folder     str     General output folder
    -- auto_capture      bool    True: automatic capture on enter
    
    <= done                     Capture completed
    <= failed                   Error
    """
    
    def __init__(self, total_poses, camera_type, base_frame='base_link', 
                 tool_frame='tool0', pictures_folder=None, robot_poses_folder=None,
                 output_folder=None, auto_capture=False):
        super().__init__(outcomes=['done', 'failed'])
        
        self.total_poses = total_poses
        self.camera_type = camera_type
        self.base_frame = base_frame
        self.tool_frame = tool_frame
        self.auto_capture = auto_capture
        self.poses_taken = 0
        
        # Configure folders
        base_path = os.path.expanduser('~/drims_ws/calibrations')
        self.output_folder = output_folder or os.path.join(base_path, 'extrinsic_calib_charuco_poses')
        self.pictures_folder = pictures_folder or os.path.join(base_path, 'extrinsic_calibration', 'pictures')
        self.robot_poses_folder = robot_poses_folder or os.path.join(base_path, 'extrinsic_calibration', 'robot_poses')
        
        # Create folders
        for folder in [self.pictures_folder, self.robot_poses_folder, self.output_folder]:
            os.makedirs(folder, exist_ok=True)
        
        # Camera components
        self.pipeline = None
        self.capture = None
        self.color_image = None
        self.window_created = False
        self.window_name = 'UR5e CALIBRATION - Pose capture'
        self.camera_initialized = False
        self.should_exit = False
        
        # TF2
        self.tf_buffer = Buffer()
        self.tf_listener = None
        
        Logger.loginfo("="*60)
        Logger.loginfo("📸 STATE: TakePoseAndPicture (UR5e)")
        Logger.loginfo("="*60)
        Logger.loginfo(f"🎯 Target: {self.total_poses} poses")
        Logger.loginfo(f"📁 Images: {self.pictures_folder}")
        Logger.loginfo(f"📁 Poses: {self.robot_poses_folder}")
        Logger.loginfo(f"📁 Output: {self.output_folder}")
        Logger.loginfo(f"🔧 Frames: {self.base_frame} → {self.tool_frame}")
        
    def on_start(self):
        """Initialize: clean folders and prepare camera"""
        self._clean_folders()
        
        # Initialize TF2
        self.tf_listener = TransformListener(self.tf_buffer, self._node, spin_thread=True)
        
        # Wait for TF to be available
        Logger.loginfo("⏳ Waiting for robot transforms...")
        time.sleep(2)
        
        self.camera_initialized = self._init_camera()
        
        if not self.camera_initialized:
            Logger.logerr("❌ Could not initialize camera")
    
    def _clean_folders(self):
        """Cleans existing files"""
        try:
            # Clean images
            for ext in ['*.jpg', '*.jpeg', '*.png']:
                for f in glob.glob(os.path.join(self.pictures_folder, ext)):
                    os.remove(f)
            
            # Clean poses
            for f in glob.glob(os.path.join(self.robot_poses_folder, 'pose_*.yaml')):
                os.remove(f)
            for f in glob.glob(os.path.join(self.robot_poses_folder, 'pose_*.txt')):
                os.remove(f)
            
            Logger.loginfo("🧹 Folders cleaned")
        except Exception as e:
            Logger.logwarn(f"⚠️ Error cleaning folders: {e}")
    
    def _init_camera(self):
        """Initializes the camera based on its type"""
        try:
            if self.camera_type == 'realsense':
                self.pipeline = rs.pipeline()
                config = rs.config()
                config.enable_stream(rs.stream.color, 1920, 1080, rs.format.bgr8, 30)
                self.pipeline.start(config)
                Logger.loginfo("✅ RealSense started (1920x1080)")
                return True
                
            elif self.camera_type == 'usb':
                Logger.loginfo("🔌 Opening USB camera...")
                self.capture = cv2.VideoCapture(0)
                self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
                self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
                
                if self.capture.isOpened():
                    # Verify real resolution
                    width = int(self.capture.get(cv2.CAP_PROP_FRAME_WIDTH))
                    height = int(self.capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
                    Logger.loginfo(f"✅ USB camera started: {width}x{height}")
                    return True
                else:
                    Logger.logerr("❌ Could not open USB camera")
                    return False
            else:
                Logger.logerr(f"❌ Unknown camera type: {self.camera_type}")
                return False
                
        except Exception as e:
            Logger.logerr(f"❌ Error starting camera: {e}")
            return False
    
    def _get_robot_pose(self):
        """Obtains current robot pose from TF"""
        try:
            # For UR5e, tool0 frame must be published
            if not self.tf_buffer.can_transform(
                self.base_frame, 
                self.tool_frame, 
                rclpy.time.Time(),
                timeout=Duration(seconds=1.0)
            ):
                # List available frames for debug
                frames = self.tf_buffer.all_frames_as_string()
                Logger.logwarn(f"Available frames: {frames[:200]}...")
                return None
            
            transform = self.tf_buffer.lookup_transform(
                self.base_frame, 
                self.tool_frame, 
                rclpy.time.Time()
            )
            return transform
            
        except Exception as e:
            Logger.logerr(f"❌ Error getting robot pose: {e}")
            return None
    
    def _save_pose(self, transform, index):
        """Saves the pose in YAML and TXT"""
        pose_data = {
            'index': index,
            'timestamp': time.time(),
            'frame_id': f"{self.base_frame}_to_{self.tool_frame}",
            'position': [
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z
            ],
            'orientation': [
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w
            ]
        }
        
        # Save YAML
        yaml_path = os.path.join(self.robot_poses_folder, f"pose_{index:03d}.yaml")
        with open(yaml_path, 'w') as f:
            yaml.dump(pose_data, f, default_flow_style=False, sort_keys=False)
        
        # Save TXT (simple format for compatibility)
        txt_path = os.path.join(self.robot_poses_folder, f"pose_{index:03d}.txt")
        with open(txt_path, 'w') as f:
            f.write(f"{pose_data['position'][0]:.6f} {pose_data['position'][1]:.6f} {pose_data['position'][2]:.6f} ")
            f.write(f"{pose_data['orientation'][0]:.6f} {pose_data['orientation'][1]:.6f} ")
            f.write(f"{pose_data['orientation'][2]:.6f} {pose_data['orientation'][3]:.6f}")
        
        Logger.loginfo(f"   💾 Pose saved: {os.path.basename(yaml_path)}")
        
        return yaml_path, txt_path
    
    def _perform_capture(self):
        """Executes image and pose capture"""
        # Get robot pose
        robot_pose = self._get_robot_pose()
        if robot_pose is None:
            Logger.logwarn("⚠️ Could not get robot pose, not saving")
            return False
        
        # Save image
        img_file = os.path.join(self.pictures_folder, f"image_{self.poses_taken + 1:03d}.jpg")
        cv2.imwrite(img_file, self.color_image)
        
        # Save pose
        yaml_file, txt_file = self._save_pose(robot_pose, self.poses_taken + 1)
        
        self.poses_taken += 1
        Logger.loginfo(f"✅ CAPTURE {self.poses_taken}/{self.total_poses}")
        Logger.loginfo(f"   📸 Image: {os.path.basename(img_file)}")
        
        if self.poses_taken >= self.total_poses:
            Logger.loginfo(f"🎯 Target reached: {self.total_poses} poses")
            self.should_exit = True
        
        return True
    
    def execute(self, userdata):
        """Main capture loop"""
        if self.should_exit:
            self._cleanup()
            return 'done'
        
        if not self.camera_initialized:
            return 'failed'
        
        try:
            # Get camera frame
            if self.camera_type == 'realsense':
                frames = self.pipeline.wait_for_frames(timeout_ms=5000)
                color_frame = frames.get_color_frame()
                if not color_frame:
                    return
                self.color_image = np.asanyarray(color_frame.get_data())
            else:  # USB
                ret, self.color_image = self.capture.read()
                if not ret:
                    Logger.logerr("❌ Error reading from USB camera")
                    return 'failed'
            
            # Create window if it does not exist
            if not self.window_created:
                cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
                cv2.resizeWindow(self.window_name, 1280, 720)
                self.window_created = True
            
            # Prepare display
            display = self.color_image.copy()
            remaining = self.total_poses - self.poses_taken
            
            # Information overlay
            h, w = display.shape[:2]
            overlay = display.copy()
            cv2.rectangle(overlay, (10, 10), (w-10, 220), (0, 0, 0), -1)
            cv2.addWeighted(overlay, 0.6, display, 0.4, 0, display)
            
            # Informative text
            cv2.putText(display, f"🤖 UR5e - EXTRINSIC CALIBRATION", 
                       (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 0), 2)
            cv2.putText(display, f"📸 CAPTURED: {self.poses_taken}/{self.total_poses}", 
                       (50, 100), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 0), 2)
            cv2.putText(display, f"⏳ REMAINING: {remaining}", 
                       (50, 150), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2)
            cv2.putText(display, f"👉 SPACE: Capture | ESC: Cancel", 
                       (50, 200), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)
            
            cv2.imshow(self.window_name, display)
            key = cv2.waitKey(1) & 0xFF
            
            # Process keys
            if key in [13, 32]:  # ENTER or SPACE
                self._perform_capture()
            elif key == 27:  # ESC
                Logger.logwarn("⏹️ Capture canceled by user")
                self._cleanup()
                return 'failed'
            
        except Exception as e:
            Logger.logerr(f"❌ Error during execution: {e}")
            self._cleanup()
            return 'failed'
    
    def _cleanup(self):
        """Resource cleanup"""
        try:
            if self.window_created:
                cv2.destroyAllWindows()
                self.window_created = False
            
            if hasattr(self, 'pipeline') and self.pipeline:
                self.pipeline.stop()
                self.pipeline = None
            
            if hasattr(self, 'capture') and self.capture:
                self.capture.release()
                self.capture = None
                
        except Exception as e:
            Logger.logwarn(f"⚠️ Error during cleanup: {e}")
    
    def on_stop(self):
        self._cleanup()
    
    def on_exit(self, userdata):
        self._cleanup()
