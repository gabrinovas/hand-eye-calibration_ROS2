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
    Versión con cálculo manual de esquinas para OpenCV 4.13.0
    """
    
    def __init__(self, square_size, marker_size, col_count, row_count, save_file_name, images_folder=None):
        super(CharucoCameraCalibrationState, self).__init__(outcomes=['done', 'failed'])
        
        self.square_size = square_size
        self.marker_size = marker_size
        self.col_count = col_count
        self.row_count = row_count
        self.save_file_name = save_file_name
        
        if images_folder:
            self.images_folder = images_folder
        else:
            self.images_folder = os.path.expanduser('~/drims_ws/calibrations/camera_calib_pictures')
        
        self.pic_folder = self.images_folder
        self.calibration_output_folder = os.path.expanduser('~/drims_ws/calibrations')
        self.final_output_path = os.path.join(self.calibration_output_folder, 'camera_intrinsics.yaml')
        
        # Diccionario
        self.dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_100)
        
        # Crear board
        self.board = aruco.CharucoBoard(
            (self.col_count, self.row_count),
            self.square_size,
            self.marker_size,
            self.dictionary
        )
        
        Logger.loginfo(f"📂 Carpeta: {self.images_folder}")
        Logger.loginfo(f"📁 Salida: {self.final_output_path}")
        Logger.loginfo(f"📋 OpenCV: {cv2.__version__}")
    
    def execute(self, userdata):
        images = glob.glob(os.path.join(self.pic_folder, '*.jpg'))
        images.extend(glob.glob(os.path.join(self.pic_folder, '*.png')))
        
        if not images:
            Logger.logerr(f"❌ No hay imágenes en {self.pic_folder}")
            return "failed"
        
        Logger.loginfo(f"🔍 Procesando {len(images)} imágenes...")
        
        all_obj_points = []  # Puntos 3D
        all_img_points = []  # Puntos 2D
        image_size = None
        
        for img_path in images:
            img = cv2.imread(img_path)
            if img is None:
                continue
                
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            
            if image_size is None:
                image_size = gray.shape[::-1]
            
            # Detectar marcadores
            corners, ids, _ = aruco.detectMarkers(gray, self.dictionary)
            
            if ids is not None and len(ids) > 10:
                # Obtener esquinas Charuco
                ret, charuco_corners, charuco_ids = aruco.interpolateCornersCharuco(
                    corners, ids, gray, self.board
                )
                
                if charuco_corners is not None and ret > 4:  # Al menos 4 esquinas
                    # Obtener puntos 3D del tablero para estas esquinas
                    obj_points = []
                    for i, corner_id in enumerate(charuco_ids):
                        # Posición 3D de la esquina en el tablero
                        point_3d = self.board.getChessboardCorners()[corner_id]
                        obj_points.append(point_3d)
                    
                    all_obj_points.append(np.array(obj_points, dtype=np.float32))
                    all_img_points.append(charuco_corners.reshape(-1, 2))
                    
                    Logger.loginfo(f"✅ {os.path.basename(img_path)}: {ret} esquinas")
                else:
                    Logger.loginfo(f"⚠️ {os.path.basename(img_path)}: Pocas esquinas ({ret})")
        
        if len(all_obj_points) < 3:
            Logger.logerr(f"❌ Solo {len(all_obj_points)} imágenes válidas (mínimo 3)")
            return "failed"
        
        Logger.loginfo(f"📊 Calibrando con {len(all_obj_points)} imágenes...")
        
        try:
            # Calibrar cámara
            ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
                all_obj_points,
                all_img_points,
                image_size,
                None,
                None
            )
            
            Logger.loginfo("\n" + "="*50)
            Logger.loginfo("✅ CALIBRACIÓN EXITOSA")
            Logger.loginfo("="*50)
            Logger.loginfo(f"📏 Error: {ret:.6f}")
            Logger.loginfo(f"\n📷 Matriz:\n{camera_matrix}")
            
            # Guardar
            calibration_data = {
                'camera_matrix': camera_matrix.tolist(),
                'distortion_coefficients': dist_coeffs.reshape(-1).tolist(),
                'image_width': image_size[0],
                'image_height': image_size[1],
                'reprojection_error': float(ret),
                'calibration_date': time.time()
            }
            
            os.makedirs(self.calibration_output_folder, exist_ok=True)
            
            with open(self.final_output_path, 'w') as f:
                yaml.dump(calibration_data, f)
            
            Logger.loginfo(f"💾 Guardado en: {self.final_output_path}")
            
            return "done"
            
        except Exception as e:
            Logger.logerr(f"❌ Error: {str(e)}")
            return "failed"
