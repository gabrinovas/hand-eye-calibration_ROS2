#!/usr/bin/env python3
from flexbe_core import EventState, Logger
import cv2
import cv2.aruco as aruco
import numpy as np
import glob
import os
import yaml
import time

class CharucoCameraCalibrationState(EventState):
    """
    Processes charuco board images to calibrate the camera.
    Version for OpenCV 4.5.4 (ROS2 Humble)
    
    <= done                                    Successful calibration
    <= failed                                   Calibration error
    """
    
    def __init__(self, square_size, marker_size, col_count, row_count, save_file_name, images_folder=None):
        """Constructor"""
        super(CharucoCameraCalibrationState, self).__init__(outcomes=['done', 'failed'])
        
        self.square_size = square_size
        self.marker_size = marker_size
        self.col_count = col_count
        self.row_count = row_count
        self.save_file_name = save_file_name
        
        # Determine images folder
        if images_folder:
            self.images_folder = images_folder
        else:
            self.images_folder = os.path.expanduser('~/calibrations/camera_calib_pictures')
        
        self.pic_folder = self.images_folder
        self.calibration_output_folder = os.path.expanduser('~/calibrations')
        self.final_output_path = os.path.join(self.calibration_output_folder, 'camera_intrinsics.yaml')
        
        # OpenCV 4.5.4 API (the one that works with Charuco)
        self.dictionary = aruco.Dictionary_get(aruco.DICT_4X4_100)
        self.board = aruco.CharucoBoard_create(
            self.col_count, self.row_count,
            self.square_size, self.marker_size,
            self.dictionary
        )
        
        Logger.loginfo(f"📂 Images folder: {self.images_folder}")
        Logger.loginfo(f"📸 Searching for images in: {self.pic_folder}")
        Logger.loginfo(f"📁 Final calibration file: {self.final_output_path}")
        Logger.loginfo(f"📋 OpenCV version: {cv2.__version__}")
        Logger.loginfo(f"📏 Board: {self.col_count}x{self.row_count}, square={self.square_size}m, marker={self.marker_size}m")
    
    def save_intrinsic_matrix_yaml(self, camera_matrix, dist_coeffs, image_size, robot_id=1, robot_name="ur5e", robot_ip="192.168.1.101"):
        """
        Saves the intrinsic matrix in the specific YAML format.
        """
        # 1. Prepare flat lists
        K_flat = camera_matrix.flatten().tolist()
        d_flat = dist_coeffs.flatten().tolist()
        
        # 2. Data structure
        intrinsic_data = {
            'camera': {
                robot_id: {
                    'K': K_flat,
                    'd': d_flat,
                }
            }
        }
        
        intrinsic_path = os.path.join(self.calibration_output_folder, 'intrinsic_matrix.yaml')
        with open(intrinsic_path, 'w') as f:
            yaml.dump(intrinsic_data, f, default_flow_style=None, width=float('inf'), sort_keys=False)
        
        print(f"💾 Saved successfully to: {intrinsic_path}")
    
    def on_start(self):
        pass
    
    def execute(self, userdata):
        # Verify that images exist
        images = glob.glob(os.path.join(self.pic_folder, '*.jpg'))
        images.extend(glob.glob(os.path.join(self.pic_folder, '*.png')))
        
        if not images:
            Logger.logerr(f"❌ No images found in: {self.pic_folder}")
            return "failed"
        
        Logger.loginfo(f"🔍 Processing {len(images)} images...")
        
        all_corners = []
        all_ids = []
        image_size = None
        valid_count = 0
        
        for img_path in images:
            img = cv2.imread(img_path)
            if img is None:
                continue
            
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            
            if image_size is None:
                image_size = gray.shape[::-1]
            
            # Detect ArUco markers
            corners, ids, _ = aruco.detectMarkers(gray, self.dictionary)
            
            if ids is not None and len(ids) > 10:
                # Interpolate Charuco corners
                ret, charuco_corners, charuco_ids = aruco.interpolateCornersCharuco(
                    corners, ids, gray, self.board
                )
                
                if charuco_corners is not None and len(charuco_corners) > 4:
                    all_corners.append(charuco_corners)
                    all_ids.append(charuco_ids)
                    valid_count += 1
                    Logger.loginfo(f"✅ {os.path.basename(img_path)}: {len(charuco_corners)} corners")
                else:
                    Logger.loginfo(f"⚠️ {os.path.basename(img_path)}: few corners ({len(charuco_corners) if charuco_corners is not None else 0})")
            else:
                Logger.loginfo(f"❌ {os.path.basename(img_path)}: few markers ({len(ids) if ids is not None else 0})")
        
        if valid_count < 3:
            Logger.logerr(f"❌ Only {valid_count} valid images (minimum 3)")
            return "failed"
        
        Logger.loginfo(f"📊 Calibrating with {valid_count} valid images...")
        
        try:
            # Calibrate camera using Charuco
            ret, camera_matrix, dist_coeffs, rvecs, tvecs = aruco.calibrateCameraCharuco(
                charucoCorners=all_corners,
                charucoIds=all_ids,
                board=self.board,
                imageSize=image_size,
                cameraMatrix=None,
                distCoeffs=None
            )
            
            Logger.loginfo("\n" + "="*50)
            Logger.loginfo("✅ SUCCESSFUL CALIBRATION")
            Logger.loginfo("="*50)
            Logger.loginfo(f"📏 Reprojection error: {ret:.6f}")
            Logger.loginfo(f"\n📷 Camera matrix:\n{camera_matrix}")
            Logger.loginfo(f"\n📐 Distortion coefficients:\n{dist_coeffs.reshape(-1)}")
            
            # Save standard calibration
            calibration_data = {
                'camera_matrix': camera_matrix.tolist(),
                'distortion_coefficients': dist_coeffs.reshape(-1).tolist(),
                'image_width': image_size[0],
                'image_height': image_size[1],
                'reprojection_error': float(ret),
                'calibration_date': time.time(),
                'charuco_config': {
                    'rows': self.row_count,
                    'cols': self.col_count,
                    'square_length': self.square_size,
                    'marker_length': self.marker_size,
                    'dictionary': 'DICT_4X4_100'
                }
            }
            
            # Ensure folder exists
            os.makedirs(self.calibration_output_folder, exist_ok=True)
            
            # Save standard YAML file
            with open(self.final_output_path, 'w') as f:
                yaml.dump(calibration_data, f, default_flow_style=None, width=float('inf'))
            
            Logger.loginfo(f"💾 Standard calibration saved to: {self.final_output_path}")
            
            # Save intrinsic matrix in specific format
            self.save_intrinsic_matrix_yaml(
                camera_matrix=camera_matrix,
                dist_coeffs=dist_coeffs,
                image_size=image_size,
                robot_id=1,
                robot_name="ur5e",
                robot_ip="192.168.1.101"
            )
            
            return "done"
            
        except Exception as e:
            Logger.logerr(f"❌ Error during calibration: {str(e)}")
            import traceback
            traceback.print_exc()
            return "failed"
    
    def on_exit(self, userdata):
        """Clean up on exit"""
        pass
