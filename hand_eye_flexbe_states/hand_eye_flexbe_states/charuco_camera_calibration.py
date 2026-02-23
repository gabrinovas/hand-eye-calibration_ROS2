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
    Procesa imágenes de charuco board para calibrar la cámara.
    Versión para OpenCV 4.5.4 (ROS2 Humble)
    
    <= done                                    Calibración exitosa
    <= failed                                   Error en calibración
    """
    
    def __init__(self, square_size, marker_size, col_count, row_count, save_file_name, images_folder=None):
        """Constructor"""
        super(CharucoCameraCalibrationState, self).__init__(outcomes=['done', 'failed'])
        
        self.square_size = square_size
        self.marker_size = marker_size
        self.col_count = col_count
        self.row_count = row_count
        self.save_file_name = save_file_name
        
        # Determinar carpeta de imágenes
        if images_folder:
            self.images_folder = images_folder
        else:
            self.images_folder = os.path.expanduser('~/drims_ws/calibrations/camera_calib_pictures')
        
        self.pic_folder = self.images_folder
        self.calibration_output_folder = os.path.expanduser('~/drims_ws/calibrations')
        self.final_output_path = os.path.join(self.calibration_output_folder, 'camera_intrinsics.yaml')
        
        # API de OpenCV 4.5.4 (la que funciona con Charuco)
        self.dictionary = aruco.Dictionary_get(aruco.DICT_4X4_100)
        self.board = aruco.CharucoBoard_create(
            self.col_count, self.row_count,
            self.square_size, self.marker_size,
            self.dictionary
        )
        
        Logger.loginfo(f"📂 Carpeta de imágenes: {self.images_folder}")
        Logger.loginfo(f"📸 Buscando imágenes en: {self.pic_folder}")
        Logger.loginfo(f"📁 Archivo final de calibración: {self.final_output_path}")
        Logger.loginfo(f"📋 OpenCV versión: {cv2.__version__}")
        Logger.loginfo(f"📏 Tablero: {self.col_count}x{self.row_count}, square={self.square_size}m, marker={self.marker_size}m")
    
    def save_intrinsic_matrix_yaml(self, camera_matrix, dist_coeffs, image_size, robot_id=1, robot_name="ur5e", robot_ip="192.168.1.101"):
        """
        Guarda la matriz intrínseca en el formato YAML específico.
        """
        # 1. Preparar las listas planas
        K_flat = camera_matrix.flatten().tolist()
        d_flat = dist_coeffs.flatten().tolist()
        
        # 2. Estructura de datos
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
        
        print(f"💾 Guardado correctamente en: {intrinsic_path}")
    
    def on_start(self):
        pass
    
    def execute(self, userdata):
        # Verificar que existen imágenes
        images = glob.glob(os.path.join(self.pic_folder, '*.jpg'))
        images.extend(glob.glob(os.path.join(self.pic_folder, '*.png')))
        
        if not images:
            Logger.logerr(f"❌ No se encontraron imágenes en: {self.pic_folder}")
            return "failed"
        
        Logger.loginfo(f"🔍 Procesando {len(images)} imágenes...")
        
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
            
            # Detectar marcadores ArUco
            corners, ids, _ = aruco.detectMarkers(gray, self.dictionary)
            
            if ids is not None and len(ids) > 10:
                # Interpolar esquinas Charuco
                ret, charuco_corners, charuco_ids = aruco.interpolateCornersCharuco(
                    corners, ids, gray, self.board
                )
                
                if charuco_corners is not None and len(charuco_corners) > 4:
                    all_corners.append(charuco_corners)
                    all_ids.append(charuco_ids)
                    valid_count += 1
                    Logger.loginfo(f"✅ {os.path.basename(img_path)}: {len(charuco_corners)} esquinas")
                else:
                    Logger.loginfo(f"⚠️ {os.path.basename(img_path)}: pocas esquinas ({len(charuco_corners) if charuco_corners is not None else 0})")
            else:
                Logger.loginfo(f"❌ {os.path.basename(img_path)}: pocos marcadores ({len(ids) if ids is not None else 0})")
        
        if valid_count < 3:
            Logger.logerr(f"❌ Solo {valid_count} imágenes válidas (mínimo 3)")
            return "failed"
        
        Logger.loginfo(f"📊 Calibrando con {valid_count} imágenes válidas...")
        
        try:
            # Calibrar cámara usando Charuco
            ret, camera_matrix, dist_coeffs, rvecs, tvecs = aruco.calibrateCameraCharuco(
                charucoCorners=all_corners,
                charucoIds=all_ids,
                board=self.board,
                imageSize=image_size,
                cameraMatrix=None,
                distCoeffs=None
            )
            
            Logger.loginfo("\n" + "="*50)
            Logger.loginfo("✅ CALIBRACIÓN EXITOSA")
            Logger.loginfo("="*50)
            Logger.loginfo(f"📏 Error de reproyección: {ret:.6f}")
            Logger.loginfo(f"\n📷 Matriz de cámara:\n{camera_matrix}")
            Logger.loginfo(f"\n📐 Coeficientes de distorsión:\n{dist_coeffs.reshape(-1)}")
            
            # Guardar calibración estándar
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
            
            # Asegurar que la carpeta existe
            os.makedirs(self.calibration_output_folder, exist_ok=True)
            
            # Guardar archivo YAML estándar
            with open(self.final_output_path, 'w') as f:
                yaml.dump(calibration_data, f, default_flow_style=False)
            
            Logger.loginfo(f"💾 Calibración estándar guardada en: {self.final_output_path}")
            
            # Guardar intrinsic matrix en formato específico
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
            Logger.logerr(f"❌ Error durante calibración: {str(e)}")
            import traceback
            traceback.print_exc()
            return "failed"
    
    def on_exit(self, userdata):
        """Limpiar al salir"""
        pass
