#!/usr/bin/env python3
"""
Estado FlexBE para capturar imágenes y poses del robot simultáneamente.
Adaptado para UR5e.
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
    Captura imágenes y poses del robot simultáneamente.
    
    -- total_poses       int     Número TOTAL de poses a capturar
    -- camera_type       str     'realsense' o 'usb'
    -- base_frame        str     Frame base del robot (base_link)
    -- tool_frame        str     Frame del efector (tool0)
    -- pictures_folder   str     Carpeta para imágenes
    -- robot_poses_folder str    Carpeta para poses
    -- output_folder     str     Carpeta general de salida
    -- auto_capture      bool    True: captura automática al entrar
    
    <= done                     Captura completada
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
        
        # Configurar carpetas
        base_path = os.path.expanduser('~/drims_ws/calibrations')
        self.output_folder = output_folder or os.path.join(base_path, 'extrinsic_calib_charuco_poses')
        self.pictures_folder = pictures_folder or os.path.join(base_path, 'extrinsic_calibration', 'pictures')
        self.robot_poses_folder = robot_poses_folder or os.path.join(base_path, 'extrinsic_calibration', 'robot_poses')
        
        # Crear carpetas
        for folder in [self.pictures_folder, self.robot_poses_folder, self.output_folder]:
            os.makedirs(folder, exist_ok=True)
        
        # Componentes de cámara
        self.pipeline = None
        self.capture = None
        self.color_image = None
        self.window_created = False
        self.window_name = 'CALIBRACIÓN UR5e - Captura de poses'
        self.camera_initialized = False
        self.should_exit = False
        
        # TF2
        self.tf_buffer = Buffer()
        self.tf_listener = None
        
        Logger.loginfo("="*60)
        Logger.loginfo("📸 ESTADO: TakePoseAndPicture (UR5e)")
        Logger.loginfo("="*60)
        Logger.loginfo(f"🎯 Objetivo: {self.total_poses} poses")
        Logger.loginfo(f"📁 Imágenes: {self.pictures_folder}")
        Logger.loginfo(f"📁 Poses: {self.robot_poses_folder}")
        Logger.loginfo(f"📁 Output: {self.output_folder}")
        Logger.loginfo(f"🔧 Frames: {self.base_frame} → {self.tool_frame}")
        
    def on_start(self):
        """Inicializar: limpiar carpetas y preparar cámara"""
        self._clean_folders()
        
        # Inicializar TF2
        self.tf_listener = TransformListener(self.tf_buffer, self._node, spin_thread=True)
        
        # Esperar a que TF esté disponible
        Logger.loginfo("⏳ Esperando transforms del robot...")
        time.sleep(2)
        
        self.camera_initialized = self._init_camera()
        
        if not self.camera_initialized:
            Logger.logerr("❌ No se pudo inicializar la cámara")
    
    def _clean_folders(self):
        """Limpia archivos existentes"""
        try:
            # Limpiar imágenes
            for ext in ['*.jpg', '*.jpeg', '*.png']:
                for f in glob.glob(os.path.join(self.pictures_folder, ext)):
                    os.remove(f)
            
            # Limpiar poses
            for f in glob.glob(os.path.join(self.robot_poses_folder, 'pose_*.yaml')):
                os.remove(f)
            for f in glob.glob(os.path.join(self.robot_poses_folder, 'pose_*.txt')):
                os.remove(f)
            
            Logger.loginfo("🧹 Carpetas limpiadas")
        except Exception as e:
            Logger.logwarn(f"⚠️ Error limpiando carpetas: {e}")
    
    def _init_camera(self):
        """Inicializa la cámara según el tipo"""
        try:
            if self.camera_type == 'realsense':
                self.pipeline = rs.pipeline()
                config = rs.config()
                config.enable_stream(rs.stream.color, 1920, 1080, rs.format.bgr8, 30)
                self.pipeline.start(config)
                Logger.loginfo("✅ RealSense iniciada (1920x1080)")
                return True
                
            elif self.camera_type == 'usb':
                Logger.loginfo("🔌 Abriendo cámara USB...")
                self.capture = cv2.VideoCapture(0)
                self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
                self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
                
                if self.capture.isOpened():
                    # Verificar resolución real
                    width = int(self.capture.get(cv2.CAP_PROP_FRAME_WIDTH))
                    height = int(self.capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
                    Logger.loginfo(f"✅ Cámara USB iniciada: {width}x{height}")
                    return True
                else:
                    Logger.logerr("❌ No se pudo abrir cámara USB")
                    return False
            else:
                Logger.logerr(f"❌ Tipo de cámara desconocido: {self.camera_type}")
                return False
                
        except Exception as e:
            Logger.logerr(f"❌ Error iniciando cámara: {e}")
            return False
    
    def _get_robot_pose(self):
        """Obtiene la pose actual del robot desde TF"""
        try:
            # Para UR5e, el frame tool0 debe estar publicado
            if not self.tf_buffer.can_transform(
                self.base_frame, 
                self.tool_frame, 
                rclpy.time.Time(),
                timeout=Duration(seconds=1.0)
            ):
                # Listar frames disponibles para debug
                frames = self.tf_buffer.all_frames_as_string()
                Logger.logwarn(f"Frames disponibles: {frames[:200]}...")
                return None
            
            transform = self.tf_buffer.lookup_transform(
                self.base_frame, 
                self.tool_frame, 
                rclpy.time.Time()
            )
            return transform
            
        except Exception as e:
            Logger.logerr(f"❌ Error obteniendo pose del robot: {e}")
            return None
    
    def _save_pose(self, transform, index):
        """Guarda la pose en YAML y TXT"""
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
        
        # Guardar YAML
        yaml_path = os.path.join(self.robot_poses_folder, f"pose_{index:03d}.yaml")
        with open(yaml_path, 'w') as f:
            yaml.dump(pose_data, f, default_flow_style=False, sort_keys=False)
        
        # Guardar TXT (formato simple para compatibilidad)
        txt_path = os.path.join(self.robot_poses_folder, f"pose_{index:03d}.txt")
        with open(txt_path, 'w') as f:
            f.write(f"{pose_data['position'][0]:.6f} {pose_data['position'][1]:.6f} {pose_data['position'][2]:.6f} ")
            f.write(f"{pose_data['orientation'][0]:.6f} {pose_data['orientation'][1]:.6f} ")
            f.write(f"{pose_data['orientation'][2]:.6f} {pose_data['orientation'][3]:.6f}")
        
        Logger.loginfo(f"   💾 Pose guardada: {os.path.basename(yaml_path)}")
        
        return yaml_path, txt_path
    
    def _perform_capture(self):
        """Ejecuta la captura de imagen y pose"""
        # Obtener pose del robot
        robot_pose = self._get_robot_pose()
        if robot_pose is None:
            Logger.logwarn("⚠️ No se pudo obtener pose del robot, no se guarda")
            return False
        
        # Guardar imagen
        img_file = os.path.join(self.pictures_folder, f"image_{self.poses_taken + 1:03d}.jpg")
        cv2.imwrite(img_file, self.color_image)
        
        # Guardar pose
        yaml_file, txt_file = self._save_pose(robot_pose, self.poses_taken + 1)
        
        self.poses_taken += 1
        Logger.loginfo(f"✅ CAPTURA {self.poses_taken}/{self.total_poses}")
        Logger.loginfo(f"   📸 Imagen: {os.path.basename(img_file)}")
        
        if self.poses_taken >= self.total_poses:
            Logger.loginfo(f"🎯 Objetivo alcanzado: {self.total_poses} poses")
            self.should_exit = True
        
        return True
    
    def execute(self, userdata):
        """Bucle principal de captura"""
        if self.should_exit:
            self._cleanup()
            return 'done'
        
        if not self.camera_initialized:
            return 'failed'
        
        try:
            # Obtener frame de cámara
            if self.camera_type == 'realsense':
                frames = self.pipeline.wait_for_frames(timeout_ms=5000)
                color_frame = frames.get_color_frame()
                if not color_frame:
                    return
                self.color_image = np.asanyarray(color_frame.get_data())
            else:  # USB
                ret, self.color_image = self.capture.read()
                if not ret:
                    Logger.logerr("❌ Error leyendo de cámara USB")
                    return 'failed'
            
            # Crear ventana si no existe
            if not self.window_created:
                cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
                cv2.resizeWindow(self.window_name, 1280, 720)
                self.window_created = True
            
            # Preparar visualización
            display = self.color_image.copy()
            remaining = self.total_poses - self.poses_taken
            
            # Overlay de información
            h, w = display.shape[:2]
            overlay = display.copy()
            cv2.rectangle(overlay, (10, 10), (w-10, 220), (0, 0, 0), -1)
            cv2.addWeighted(overlay, 0.6, display, 0.4, 0, display)
            
            # Texto informativo
            cv2.putText(display, f"🤖 UR5e - CALIBRACIÓN EXTRÍNSECA", 
                       (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 0), 2)
            cv2.putText(display, f"📸 CAPTURADAS: {self.poses_taken}/{self.total_poses}", 
                       (50, 100), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 0), 2)
            cv2.putText(display, f"⏳ FALTAN: {remaining}", 
                       (50, 150), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2)
            cv2.putText(display, f"👉 ESPACIO: Capturar | ESC: Cancelar", 
                       (50, 200), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)
            
            cv2.imshow(self.window_name, display)
            key = cv2.waitKey(1) & 0xFF
            
            # Procesar teclas
            if key in [13, 32]:  # ENTER o ESPACIO
                self._perform_capture()
            elif key == 27:  # ESC
                Logger.logwarn("⏹️ Captura cancelada por usuario")
                self._cleanup()
                return 'failed'
            
        except Exception as e:
            Logger.logerr(f"❌ Error en ejecución: {e}")
            self._cleanup()
            return 'failed'
    
    def _cleanup(self):
        """Limpieza de recursos"""
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
            Logger.logwarn(f"⚠️ Error en limpieza: {e}")
    
    def on_stop(self):
        self._cleanup()
    
    def on_exit(self, userdata):
        self._cleanup()
