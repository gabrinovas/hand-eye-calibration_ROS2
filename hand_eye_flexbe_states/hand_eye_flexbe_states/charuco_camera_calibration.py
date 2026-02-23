#!/usr/bin/env python3
from flexbe_core import EventState, Logger
import os
import cv2
import cv2.aruco as aruco
import numpy as np
import glob
import yaml
import time
from ament_index_python.packages import get_package_share_directory

class CharucoCameraCalibrationState(EventState):
    """
    Procesa imágenes de charuco board para calibrar la cámara usando charuco_calibrator.
    
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
        
        # Carpeta para guardar resultados
        self.calibration_output_folder = os.path.expanduser('~/drims_ws/calibrations')
        self.final_output_path = os.path.join(self.calibration_output_folder, 'camera_intrinsics.yaml')
        
        # Diccionario conocido que funciona
        self.dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_100)
        
        Logger.loginfo(f"📂 Carpeta de imágenes: {self.images_folder}")
        Logger.loginfo(f"📸 Buscando imágenes en: {self.pic_folder}")
        Logger.loginfo(f"📁 Archivo final de calibración: {self.final_output_path}")
        Logger.loginfo(f"📋 Usando diccionario: DICT_4X4_100")
    
    def on_start(self):
        pass
    
    def execute(self, userdata):
        # Verificar que existen imágenes
        images = glob.glob(os.path.join(self.pic_folder, '*.jpg'))
        images.extend(glob.glob(os.path.join(self.pic_folder, '*.png')))
        
        if not images:
            Logger.logerr(f"❌ No se encontraron imágenes en: {self.pic_folder}")
            return "failed"
        
        Logger.loginfo(f"🔍 Encontradas {len(images)} imágenes")
        
        # Probar diferentes formas de crear el CharucoBoard
        board = None
        board_creator_success = False
        
        # Método 1: API estándar con tupla
        try:
            board = aruco.CharucoBoard(
                (self.col_count, self.row_count),
                self.square_size,
                self.marker_size,
                self.dictionary
            )
            Logger.loginfo("✅ Board creado con API estándar (tupla)")
            board_creator_success = True
        except Exception as e:
            Logger.logwarn(f"⚠️ API estándar falló: {str(e)}")
        
        # Método 2: API con parámetros separados (si el método 1 falló)
        if not board_creator_success:
            try:
                board = aruco.CharucoBoard(
                    self.col_count,
                    self.row_count,
                    self.square_size,
                    self.marker_size,
                    self.dictionary
                )
                Logger.loginfo("✅ Board creado con API de parámetros separados")
                board_creator_success = True
            except Exception as e:
                Logger.logwarn(f"⚠️ API parámetros separados falló: {str(e)}")
        
        # Método 3: CharucoBoard_create (si los anteriores fallaron)
        if not board_creator_success:
            try:
                board = aruco.CharucoBoard_create(
                    self.col_count,
                    self.row_count,
                    self.square_size,
                    self.marker_size,
                    self.dictionary
                )
                Logger.loginfo("✅ Board creado con CharucoBoard_create")
                board_creator_success = True
            except Exception as e:
                Logger.logerr(f"❌ Todas las APIs fallaron: {str(e)}")
                return "failed"
        
        # Recolectar datos de calibración
        all_corners = []
        all_ids = []
        image_size = None
        valid_images = 0
        
        # Probar diferentes valores de minMarkers
        min_markers_values = [2, 3, 4, 5]
        
        for img_path in images:
            img = cv2.imread(img_path)
            if img is None:
                continue
                
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            
            if image_size is None:
                image_size = gray.shape[::-1]
            
            # Detectar marcadores
            corners, ids, _ = aruco.detectMarkers(gray, self.dictionary)
            
            if ids is not None and len(ids) > 5:
                # Intentar con diferentes valores de minMarkers
                for min_markers in min_markers_values:
                    try:
                        ret, charuco_corners, charuco_ids = aruco.interpolateCornersCharuco(
                            corners, 
                            ids, 
                            gray, 
                            board,
                            minMarkers=min_markers
                        )
                        
                        if charuco_corners is not None and ret is not None and ret > 3:
                            all_corners.append(charuco_corners)
                            all_ids.append(charuco_ids)
                            valid_images += 1
                            Logger.loginfo(f"✅ Imagen válida: {os.path.basename(img_path)} ({ret} esquinas, minMarkers={min_markers})")
                            break  # Salir del bucle de minMarkers si funciona
                    except Exception as e:
                        continue
        
        if valid_images < 3:
            Logger.logerr(f"❌ No hay suficientes imágenes válidas: {valid_images}/3")
            Logger.loginfo("\n💡 Posibles soluciones:")
            Logger.loginfo("1. Asegura que el tablero esté completamente plano (pégalo en una superficie rígida)")
            Logger.loginfo("2. Mejora la iluminación (evita sombras y reflejos)")
            Logger.loginfo("3. Acerca o aleja la cámara para que los marcadores sean más visibles")
            Logger.loginfo("4. Prueba con diferentes ángulos del tablero")
            return "failed"
        
        Logger.loginfo(f"📊 Calibrando con {valid_images} imágenes válidas...")
        
        try:
            # Calibrar cámara
            ret, camera_matrix, dist_coeffs, rvecs, tvecs = aruco.calibrateCameraCharuco(
                charucoCorners=all_corners,
                charucoIds=all_ids,
                board=board,
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
            
            # Guardar calibración
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
            
            # Guardar archivo
            with open(self.final_output_path, 'w') as f:
                yaml.dump(calibration_data, f, default_flow_style=False)
            
            Logger.loginfo(f"💾 Calibración guardada en: {self.final_output_path}")
            
            return "done"
            
        except Exception as e:
            Logger.logerr(f"❌ Error durante calibración: {str(e)}")
            return "failed"
    
    def on_exit(self, userdata):
        """Limpiar al salir"""
        pass
