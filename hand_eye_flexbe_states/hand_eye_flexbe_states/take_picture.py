#!/usr/bin/env python3
from flexbe_core import EventState, Logger
import cv2
import pyrealsense2 as rs
import numpy as np
import os
import time
import glob
import threading
from ament_index_python.packages import get_package_share_directory

class TakePictureState(EventState):
    """
    Captures images using RealSense camera for calibration.
    Images are saved in the folder used by charuco_calibrator.
    
    -- pic_num          int     TOTAL number of images to capture (e.g. 30)
    -- camera_type      str     Camera type ('realsense' or 'usb')
    -- output_folder    str     Folder to save images (optional)
    
    BEHAVIOR:
    1. On start: Automatically DELETES all existing images
    2. You control: Press SPACE or ENTER to capture each image
    3. Auto-detection: When pic_num is reached, closes and proceeds to calibration
    
    <= done                     Capture completed (pic_num reached)
    <= failed                   Error or canceled by user (ESC)
    """
    
    def __init__(self, pic_num, camera_type, output_folder=None):
        super(TakePictureState, self).__init__(outcomes=['done', 'failed'])
        
        self.pic_num = pic_num
        self.camera_type = camera_type
        self.images_taken = 0
        self.pipeline = None
        self.color_image = None
        self.window_created = False
        self.window_name = 'CALIBRATION - Manual photo capture'
        self.camera_initialized = False
        self.should_exit = False
        
        if output_folder:
            self.save_pwd = output_folder
        else:
            self.save_pwd = os.path.expanduser('~/calibrations/camera_calib_pictures')
        
        # Create directory if it doesn't exist
        os.makedirs(self.save_pwd, exist_ok=True)
        
        Logger.loginfo(f"📷 Images will be saved in: {self.save_pwd}")
        Logger.loginfo(f"📸 Target: {self.pic_num} images")
        
    def on_start(self):
        """Initialize: CLEAN EVERYTHING and prepare camera"""
        
        try:
            old_images = glob.glob(os.path.join(self.save_pwd, '*.jpg'))
            if old_images:
                Logger.loginfo(f"🧹 Cleaning {len(old_images)} images from previous runs...")
                for i, img in enumerate(old_images):
                    os.remove(img)
                Logger.loginfo("✅ Cleaning completed. Folder ready for new capture.")
            else:
                Logger.loginfo("🧹 No previous images to clean. Folder is empty.")
        except Exception as e:
            Logger.logwarn(f"⚠️ Error during cleaning: {str(e)}")
        
        # Initialize camera
        self.camera_initialized = self._init_camera()
        
        if not self.camera_initialized:
            Logger.logerr("❌ Could not initialize camera. Check the connection.")
    
    def _init_camera(self):
        """Initializes the camera based on its type"""
        try:
            if self.camera_type == 'realsense':
                # Try different configurations
                try:
                    self.pipeline = rs.pipeline()
                    config = rs.config()
                    config.enable_stream(rs.stream.color, 1920, 1080, rs.format.bgr8, 30)
                    self.pipeline.start(config)
                    Logger.loginfo("✅ RealSense started successfully (1920x1080)")
                except Exception as e:
                    Logger.logwarn(f"⚠️ Could not start with 1920x1080: {str(e)}")
                    self.pipeline = rs.pipeline()
                    config = rs.config()
                    config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
                    self.pipeline.start(config)
                    Logger.loginfo("✅ RealSense started successfully (1280x720)")
                
                time.sleep(1.0)
                active_profile = self.pipeline.get_active_profile()
                color_stream = active_profile.get_stream(rs.stream.color)
                intrinsics = color_stream.as_video_stream_profile().get_intrinsics()
                Logger.loginfo(f"📷 Resolution: {intrinsics.width}x{intrinsics.height}")
                return True
                
            elif self.camera_type == 'usb':
                Logger.loginfo("🔌 Attempting to open USB camera...")
                self.capture = cv2.VideoCapture(0)
                if not self.capture.isOpened():
                    Logger.logerr("❌ Could not open USB camera")
                    return False
                
                self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
                self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
                
                ret, frame = self.capture.read()
                if not ret:
                    Logger.logerr("❌ Could not read test frame")
                    return False
                
                Logger.loginfo("✅ USB camera started successfully")
                return True
                
        except Exception as e:
            Logger.logerr(f"❌ Error starting camera: {str(e)}")
            return False
    
    def execute(self, userdata):
        """Main loop"""
        
        # If we should exit, clean up and return done
        if self.should_exit:
            self._nuclear_cleanup()
            return 'done'
        
        if not self.camera_initialized:
            Logger.logerr("❌ Camera not available. Aborting.")
            return 'failed'
        
        try:
            # Get frame
            if self.camera_type == 'realsense':
                try:
                    frames = self.pipeline.wait_for_frames(timeout_ms=5000)
                    color_frame = frames.get_color_frame()
                    
                    if not color_frame:
                        Logger.logwarn("⏳ Waiting for frames...")
                        return
                    
                    self.color_image = np.asanyarray(color_frame.get_data())
                    
                except RuntimeError as e:
                    Logger.logerr(f"❌ RealSense error: {str(e)}")
                    return 'failed'
                    
            else:  # USB
                ret, self.color_image = self.capture.read()
                if not ret:
                    Logger.logerr("❌ Error reading from USB camera")
                    return 'failed'
            
            # Create window
            if not self.window_created:
                cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
                cv2.resizeWindow(self.window_name, 1280, 720)
                self.window_created = True
                Logger.loginfo("🪟 Capture window created")
            
            # Prepare image
            display_image = self.color_image.copy()
            remaining = self.pic_num - self.images_taken
            
            # Information overlay
            h, w = display_image.shape[:2]
            overlay = display_image.copy()
            cv2.rectangle(overlay, (10, 10), (w-10, 220), (0, 0, 0), -1)
            cv2.addWeighted(overlay, 0.5, display_image, 0.5, 0, display_image)
            
            cv2.putText(display_image, f"📸 CAPTURED: {self.images_taken}/{self.pic_num}", 
                       (50, 60), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 255, 0), 3)
            cv2.putText(display_image, f"⏳ REMAINING: {remaining}", 
                       (50, 120), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 255), 2)
            cv2.putText(display_image, f"📁 Saving in: {os.path.basename(self.save_pwd)}", 
                       (50, 170), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
            cv2.putText(display_image, "👉 SPACE or ENTER: Take photo | ESC: Cancel", 
                       (50, 220), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
            
            cv2.imshow(self.window_name, display_image)
            key = cv2.waitKey(1) & 0xFF
            
            if key == 13 or key == 32:  # ENTER or SPACE
                
                filename = os.path.join(self.save_pwd, f"camera-pic-of-charucoboard-{self.images_taken + 1:02d}.jpg")
                
                cv2.imwrite(filename, self.color_image)
                self.images_taken += 1
                
                Logger.loginfo(f"✅ PHOTO {self.images_taken}/{self.pic_num} saved: {os.path.basename(filename)}")
                
                # If it is the last photo
                if self.images_taken >= self.pic_num:
                    Logger.loginfo(f"🎯 TARGET REACHED! {self.pic_num} images")
                    Logger.loginfo("🔄 Closing window and proceeding to calibration...")
                    
                    # Flag that we should exit
                    self.should_exit = True
                    
                    # Nuclear window closing
                    self._nuclear_cleanup()
                    
                    return 'done'
                    
            elif key == 27:  # ESC
                Logger.logwarn("⏹️ Capture canceled")
                self._nuclear_cleanup()
                return 'failed'
            
        except Exception as e:
            Logger.logerr(f"❌ Error: {str(e)}")
            self._nuclear_cleanup()
            return 'failed'
        
        return
    
    def _nuclear_cleanup(self):
        """NUCLEAR CLEANUP - closes everything ruthlessly"""
        try:
            # Close ALL OpenCV windows
            cv2.destroyAllWindows()
            for _ in range(5):  # Multiple attempts
                cv2.waitKey(1)
            
            self.window_created = False
            Logger.loginfo("💥 Windows closed (nuclear mode)")
            
            # Stop RealSense
            if hasattr(self, 'pipeline') and self.pipeline is not None:
                try:
                    self.pipeline.stop()
                    Logger.loginfo("🛑 RealSense stopped")
                except:
                    pass
                finally:
                    self.pipeline = None
            
            # Release USB camera
            if hasattr(self, 'capture') and self.capture is not None:
                try:
                    self.capture.release()
                    Logger.loginfo("🛑 USB camera released")
                except:
                    pass
                finally:
                    self.capture = None
                
        except Exception as e:
            Logger.logwarn(f"Error in nuclear cleanup: {str(e)}")
    
    def on_stop(self):
        self._nuclear_cleanup()
    
    def on_exit(self, userdata):
        self._nuclear_cleanup()
