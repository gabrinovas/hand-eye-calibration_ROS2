#!/usr/bin/env python3
from flexbe_core import EventState, Logger
import cv2
import pyrealsense2 as rs
import numpy as np
import os
import time
import glob
import yaml
from geometry_msgs.msg import TransformStamped
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import rclpy
from rclpy.duration import Duration

class TakePoseAndPictureState(EventState):
    """
    Captura imágenes y poses del robot simultáneamente.
    
    -- total_poses       int     Número TOTAL de poses a capturar
    -- camera_type       str     'realsense' o 'usb'
    -- base_frame        str     Frame base del robot
    -- tool_frame        str     Frame del efector
    -- pictures_folder   str     Carpeta para imágenes
    -- robot_poses_folder str    Carpeta para poses
    -- auto_capture      bool    True: captura automática al entrar
    
    <= done                     Captura completada
    <= failed                   Error
    """
    
    def __init__(self, total_poses, camera_type, base_frame='base_link', 
                 tool_frame='tool0', pictures_folder=None, robot_poses_folder=None,
                 auto_capture=False):
        super().__init__(outcomes=['done', 'failed'])
        
        self.total_poses = total_poses
        self.camera_type = camera_type
        self.base_frame = base_frame
        self.tool_frame = tool_frame
        self.auto_capture = auto_capture
        self.poses_taken = 0
        
        # Configurar carpetas
        base_path = os.path.expanduser('~/drims_ws/calibrations/extrinsic_calibration')
        self.pictures_folder = pictures_folder or os.path.join(base_path, 'pictures')
        self.robot_poses_folder = robot_poses_folder or os.path.join(base_path, 'robot_poses')
        
        os.makedirs(self.pictures_folder, exist_ok=True)
        os.makedirs(self.robot_poses_folder, exist_ok=True)
        
        # Componentes de cámara
        self.pipeline = None
        self.capture = None
        self.color_image = None
        self.window_created = False
        self.window_name = 'CALIBRACIÓN - Captura de poses'
        self.camera_initialized = False
        self.should_exit = False
        
        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = None
        
        Logger.loginfo("="*60)
        Logger.loginfo("📸 ESTADO: TakePoseAndPicture")
        Logger.loginfo(f"🎯 Objetivo: {self.total_poses} poses")
        Logger.loginfo(f"📁 Imágenes: {self.pictures_folder}")
        Logger.loginfo(f"📁 Poses: {self.robot_poses_folder}")
        
    def on_start(self):
        """Inicializar: limpiar carpetas y preparar cámara"""
        self._clean_folders()
        self.tf_listener = TransformListener(self.tf_buffer, self._node)
        self.camera_initialized = self._init_camera()
        
        if not self.camera_initialized:
            Logger.logerr("❌ No se pudo inicializar la cámara")
    
    def _clean_folders(self):
        """Limpia archivos existentes"""
        try:
            for ext in ['*.jpg', '*.jpeg', '*.png']:
                for f in glob.glob(os.path.join(self.pictures_folder, ext)):
                    os.remove(f)
            for ext in ['*.yaml', '*.txt']:
                for f in glob.glob(os.path.join(self.robot_poses_folder, ext)):
                    os.remove(f)
            Logger.loginfo("🧹 Carpetas limpiadas")
        except Exception as e:
            Logger.logwarn(f"⚠️ Error limpiando: {e}")
    
    def _init_camera(self):
        """Inicializa la cámara"""
        try:
            if self.camera_type == 'realsense':
                self.pipeline = rs.pipeline()
                config = rs.config()
                config.enable_stream(rs.stream.color, 1920, 1080, rs.format.bgr8, 30)
                self.pipeline.start(config)
                Logger.loginfo("✅ RealSense iniciada")
                return True
            else:  # USB
                self.capture = cv2.VideoCapture(0)
                self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
                self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
                if self.capture.isOpened():
                    Logger.loginfo("✅ Cámara USB iniciada")
                    return True
        except Exception as e:
            Logger.logerr(f"❌ Error cámara: {e}")
            return False
    
    def _get_robot_pose(self):
        """Obtiene la pose actual del robot"""
        try:
            if not self.tf_buffer.can_transform(self.base_frame, self.tool_frame, 
                                                 rclpy.time.Time(), timeout=Duration(seconds=1.0)):
                return None
            
            return self.tf_buffer.lookup_transform(
                self.base_frame, self.tool_frame, rclpy.time.Time()
            )
        except Exception as e:
            Logger.logerr(f"❌ Error TF: {e}")
            return None
    
    def _save_pose(self, transform, index):
        """Guarda la pose en YAML y TXT"""
        pose_data = {
            'index': index,
            'timestamp': time.time(),
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
        
        # YAML
        with open(os.path.join(self.robot_poses_folder, f"pose_{index:03d}.yaml"), 'w') as f:
            yaml.dump(pose_data, f)
        
        # TXT
        with open(os.path.join(self.robot_poses_folder, f"pose_{index:03d}.txt"), 'w') as f:
            f.write(f"{pose_data['position'][0]} {pose_data['position'][1]} {pose_data['position'][2]} ")
            f.write(f"{pose_data['orientation'][0]} {pose_data['orientation'][1]} ")
            f.write(f"{pose_data['orientation'][2]} {pose_data['orientation'][3]}")
    
    def _perform_capture(self, userdata):
        """Ejecuta la captura"""
        # Obtener pose
        robot_pose = self._get_robot_pose()
        if robot_pose is None:
            Logger.logwarn("⚠️ No se pudo obtener pose")
            return
        
        # Guardar imagen
        img_file = os.path.join(self.pictures_folder, f"image_{self.poses_taken + 1:03d}.jpg")
        cv2.imwrite(img_file, self.color_image)
        
        # Guardar pose
        self._save_pose(robot_pose, self.poses_taken + 1)
        
        self.poses_taken += 1
        Logger.loginfo(f"✅ Captura {self.poses_taken}/{self.total_poses}")
        
        if self.poses_taken >= self.total_poses:
            self.should_exit = True
    
    def execute(self, userdata):
        if self.should_exit:
            self._cleanup()
            return 'done'
        
        if not self.camera_initialized:
            return 'failed'
        
        try:
            # Obtener frame
            if self.camera_type == 'realsense':
                frames = self.pipeline.wait_for_frames()
                color_frame = frames.get_color_frame()
                if not color_frame:
                    return
                self.color_image = np.asanyarray(color_frame.get_data())
            else:
                ret, self.color_image = self.capture.read()
                if not ret:
                    return 'failed'
            
            # Ventana
            if not self.window_created:
                cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
                cv2.resizeWindow(self.window_name, 1280, 720)
                self.window_created = True
            
            # Mostrar
            display = self.color_image.copy()
            remaining = self.total_poses - self.poses_taken
            
            cv2.putText(display, f"Capturadas: {self.poses_taken}/{self.total_poses}", 
                       (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2)
            cv2.putText(display, f"Faltan: {remaining}", 
                       (50, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,255), 2)
            cv2.putText(display, "ESPACIO: Capturar | ESC: Cancelar", 
                       (50, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255,255,255), 2)
            
            cv2.imshow(self.window_name, display)
            key = cv2.waitKey(1) & 0xFF
            
            if key in [13, 32]:  # ENTER o ESPACIO
                self._perform_capture(userdata)
            elif key == 27:  # ESC
                Logger.logwarn("⏹️ Cancelado")
                self._cleanup()
                return 'failed'
            
        except Exception as e:
            Logger.logerr(f"❌ Error: {e}")
            return 'failed'
    
    def _cleanup(self):
        cv2.destroyAllWindows()
        if self.pipeline:
            self.pipeline.stop()
        if self.capture:
            self.capture.release()
        self.window_created = False
    
    def on_stop(self):
        self._cleanup()
    
    def on_exit(self, userdata):
        self._cleanup()
