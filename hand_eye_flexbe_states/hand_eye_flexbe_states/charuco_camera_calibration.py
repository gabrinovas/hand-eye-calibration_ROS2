#!/usr/bin/env python3
from flexbe_core import EventState, Logger
import os
import subprocess
import time
import glob
import shutil
import cv2
import cv2.aruco as aruco
import numpy as np
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
        
        self.calibration_output_folder = os.path.expanduser('~/drims_ws/calibrations')
        self.final_output_path = os.path.join(self.calibration_output_folder, 'camera_intrinsics.yaml')
        
        try:
            self.config_file = os.path.join(
                get_package_share_directory('charuco_calibrator'),
                'config',
                'charuco_params.yaml'
            )
        except:
            self.config_file = os.path.expanduser('~/ros2_ws/src/charuco_calibrator/config/charuco_params.yaml')
        
        Logger.loginfo(f"📂 Carpeta de imágenes: {self.images_folder}")
        Logger.loginfo(f"📸 Buscando imágenes en: {self.pic_folder}")
        Logger.loginfo(f"⚙️ Archivo de configuración: {self.config_file}")
        Logger.loginfo(f"📁 Archivo final de calibración: {self.final_output_path}")
        
        self.temp_config = os.path.join(self.images_folder, 'temp_charuco_params.yaml')
        self._create_temp_config()

    def _create_temp_config(self):
        """Crea archivo de configuración temporal"""
        config_content = f"""charuco_calibrator:
  ros__parameters:
    charuco_rows: {self.row_count}
    charuco_cols: {self.col_count}
    square_length: {self.square_size}
    marker_length: {self.marker_size}
    dictionary: "DICT_4X4_100"
    output_file: "camera_intrinsics.yaml"
    image_width: 1920
    image_height: 1080
    min_markers: 3
    min_corners: 4
    min_valid_images: 3
"""
        try:
            with open(self.temp_config, 'w') as f:
                f.write(config_content)
            Logger.loginfo(f"✅ Archivo temporal: {self.temp_config}")
            Logger.loginfo(f"   Diccionario: DICT_4X4_100 (70 marcadores)")
        except Exception as e:
            Logger.logwarn(f"⚠️ No se pudo crear archivo temporal: {str(e)}")
    
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
        Logger.loginfo(f"📸 Leyendo imágenes de: {self.pic_folder}")
        Logger.loginfo(f"💾 Guardando calibración en: {self.calibration_output_folder}")
        
        # ===== DIAGNÓSTICO MEJORADO =====
        Logger.loginfo("🔬 INICIANDO DIAGNÓSTICO DE DETECCIÓN...")
        
        # Probar diccionarios
        dicts_to_test = [
            ("DICT_4X4_100", aruco.DICT_4X4_100),
            ("DICT_4X4_50", aruco.DICT_4X4_50),
            ("DICT_4X4_250", aruco.DICT_4X4_250),
            ("DICT_4X4_1000", aruco.DICT_4X4_1000),
        ]
        
        best_dict = None
        best_count = 0
        
        # Parámetros de detección mejorados
        detector_params = aruco.DetectorParameters()
        detector_params.adaptiveThreshWinSizeMin = 3
        detector_params.adaptiveThreshWinSizeMax = 23
        detector_params.adaptiveThreshWinSizeStep = 10
        detector_params.minMarkerPerimeterRate = 0.03
        detector_params.maxMarkerPerimeterRate = 4.0
        detector_params.polygonalApproxAccuracyRate = 0.05
        detector_params.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
        
        for dict_name, dict_id in dicts_to_test:
            Logger.loginfo(f"\n📋 Probando diccionario: {dict_name}")
            
            dictionary = aruco.getPredefinedDictionary(dict_id)
            
            # Crear board con la API que funciona
            try:
                board = aruco.CharucoBoard(
                    (self.col_count, self.row_count),
                    self.square_size,
                    self.marker_size,
                    dictionary
                )
                
                total_markers = 0
                total_charuco = 0
                valid_images = 0
                
                # Probar con las primeras 5 imágenes
                for i, img_path in enumerate(images[:5]):
                    img = cv2.imread(img_path)
                    if img is None:
                        continue
                        
                    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
                    
                    # Detectar marcadores con parámetros mejorados
                    corners, ids, _ = aruco.detectMarkers(
                        gray, 
                        dictionary,
                        parameters=detector_params
                    )
                    
                    if ids is not None and len(ids) > 3:
                        total_markers += len(ids)
                        
                        # Intentar interpolación con diferentes métodos
                        try:
                            # Método 1: Interpolación estándar
                            ret, charuco_corners, charuco_ids = aruco.interpolateCornersCharuco(
                                corners, ids, gray, board
                            )
                            
                            if charuco_corners is not None and ret is not None and ret > 0:
                                total_charuco += ret
                                valid_images += 1
                                Logger.loginfo(f"   ✅ Imagen {i+1}: {ret} esquinas detectadas")
                            else:
                                # Método 2: Intentar con minMarkers más bajo
                                ret, charuco_corners, charuco_ids = aruco.interpolateCornersCharuco(
                                    corners, ids, gray, board, minMarkers=2
                                )
                                if charuco_corners is not None and ret is not None and ret > 0:
                                    total_charuco += ret
                                    valid_images += 1
                                    Logger.loginfo(f"   ⚠️ Imagen {i+1}: {ret} esquinas (con minMarkers=2)")
                                else:
                                    Logger.loginfo(f"   ❌ Imagen {i+1}: Sin esquinas (ret={ret})")
                        except Exception as e:
                            Logger.loginfo(f"   ⚠️ Error en imagen {i+1}: {str(e)}")
                    else:
                        Logger.loginfo(f"   ❌ Imagen {i+1}: Pocos marcadores ({len(ids) if ids is not None else 0})")
                
                Logger.loginfo(f"   📊 Resultados: {valid_images}/5 válidas, {total_charuco} esquinas totales, {total_markers} marcadores")
                
                if valid_images > best_count:
                    best_count = valid_images
                    best_dict = dict_name
                    
            except Exception as e:
                Logger.loginfo(f"   ❌ Error creando board: {str(e)}")
        
        Logger.loginfo("\n" + "="*60)
        Logger.loginfo(f"🎯 MEJOR DICCIONARIO: {best_dict} con {best_count}/5 imágenes válidas")
        Logger.loginfo("="*60)
        
        if best_count == 0:
            Logger.logerr("❌ NO SE DETECTARON ESQUINAS CHARUCO")
            Logger.loginfo("\nPOSIBLES CAUSAS:")
            Logger.loginfo("1. Las dimensiones físicas del tablero no coinciden (square_size vs marker_size)")
            Logger.loginfo("2. El tablero está demasiado inclinado o borroso")
            Logger.loginfo("3. La relación de aspecto de los marcadores no es la esperada")
            Logger.loginfo("4. Prueba con square_size=0.021 o marker_size=0.014 (pequeños ajustes)")
            return "failed"
        
        # Si hay detecciones, continuar con la calibración
        Logger.loginfo(f"🚀 Iniciando calibración con {best_dict}...")
        
        try:
            # Usar el diccionario que funcionó
            self._update_temp_config_with_dict(best_dict)
            
            launch_cmd = [
                'ros2', 'launch', 'charuco_calibrator', 'charuco_detector.launch.py',
                f'images_folder:={self.pic_folder}',
                f'config_file:={self.temp_config}',
                f'output_file:=camera_intrinsics.yaml',
                'show_preview:=false'
            ]
            
            Logger.loginfo(f"📋 Ejecutando: {' '.join(launch_cmd)}")
            
            process = subprocess.Popen(
                launch_cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                universal_newlines=True,
                bufsize=1
            )
            
            calibration_success = False
            generated_file = None
            valid_detections = 0
            
            for line in process.stdout:
                Logger.loginfo(f"[charuco_calibrator] {line.strip()}")
                
                if "✅ CALIBRACIÓN EXITOSA" in line:
                    calibration_success = True
                
                if "✅ Detectadas" in line and "esquinas" in line:
                    valid_detections += 1
                
                if "💾 Calibración guardada en:" in line:
                    parts = line.strip().split("💾 Calibración guardada en: ")
                    if len(parts) > 1:
                        generated_file = parts[1].strip()
            
            process.wait()
            
            if process.returncode == 0 and calibration_success:
                Logger.loginfo("✅ Calibración exitosa")
                Logger.loginfo(f"📊 Válidas: {valid_detections}/{len(images)}")
                
                if generated_file and os.path.exists(generated_file):
                    try:
                        os.makedirs(self.calibration_output_folder, exist_ok=True)
                        
                        if os.path.exists(self.final_output_path):
                            backup = self.final_output_path.replace('.yaml', f'_backup_{int(time.time())}.yaml')
                            shutil.move(self.final_output_path, backup)
                            Logger.loginfo(f"📦 Backup: {backup}")
                        
                        shutil.copy2(generated_file, self.final_output_path)
                        Logger.loginfo(f"✅ Guardado en: {self.final_output_path}")
                        
                    except Exception as e:
                        Logger.logwarn(f"⚠️ Error al copiar: {str(e)}")
                
                return "done"
            else:
                Logger.logerr(f"❌ Calibración fallida")
                Logger.logerr(f"📊 Válidas: {valid_detections}/{len(images)}")
                return "failed"
                
        except Exception as e:
            Logger.logerr(f"❌ Error: {str(e)}")
            return "failed"
    
    def _update_temp_config_with_dict(self, dict_name):
        """Actualiza el archivo temporal con el diccionario detectado"""
        try:
            with open(self.temp_config, 'r') as f:
                content = f.read()
            
            import re
            new_content = re.sub(r'dictionary: "DICT_\w+"', f'dictionary: "{dict_name}"', content)
            
            with open(self.temp_config, 'w') as f:
                f.write(new_content)
            
            Logger.loginfo(f"✅ Diccionario actualizado a {dict_name}")
        except Exception as e:
            Logger.logwarn(f"⚠️ No se pudo actualizar diccionario: {str(e)}")
    
    def on_exit(self, userdata):
        """Limpiar archivos temporales"""
        try:
            if os.path.exists(self.temp_config):
                os.remove(self.temp_config)
                Logger.loginfo("🧹 Archivo temporal eliminado")
        except Exception as e:
            Logger.logwarn(f"⚠️ Error: {str(e)}")
