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
    min_markers: 4
    min_corners: 10
    min_valid_images: 5
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
        # Verificar que la carpeta de imágenes existe
        if not os.path.exists(self.pic_folder):
            Logger.logerr(f"❌ La carpeta no existe: {self.pic_folder}")
            return "failed"
        
        # Asegurar que la carpeta de salida existe
        os.makedirs(self.calibration_output_folder, exist_ok=True)
        
        Logger.loginfo(f"📸 Leyendo imágenes de: {os.path.abspath(self.pic_folder)}")
        Logger.loginfo(f"💾 Guardando calibración en: {os.path.abspath(self.calibration_output_folder)}")
    
    # Verificar que existen imágenes
        images = glob.glob(os.path.join(self.pic_folder, '*.jpg'))
        images.extend(glob.glob(os.path.join(self.pic_folder, '*.png')))
        
        if not images:
            Logger.logerr(f"❌ No se encontraron imágenes en: {self.pic_folder}")
            return "failed"
        
        Logger.loginfo(f"🔍 Encontradas {len(images)} imágenes")
        
        # ===== DIAGNÓSTICO VISUAL =====
        Logger.loginfo("🔬 INICIANDO DIAGNÓSTICO DE DETECCIÓN...")
        
        # Probar diferentes diccionarios
        dicts_to_test = [
            ("DICT_4X4_100", aruco.DICT_4X4_100),
            ("DICT_4X4_1000", aruco.DICT_4X4_1000),
            ("DICT_5X5_100", aruco.DICT_5X5_100),
            ("DICT_6X6_100", aruco.DICT_6X6_100),
        ]
        
        best_dict = None
        best_count = 0
        
        # Crear ventana para visualización
        cv2.namedWindow('Diagnóstico Charuco', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('Diagnóstico Charuco', 1280, 720)
        
        for dict_name, dict_id in dicts_to_test:
            Logger.loginfo(f"\n📋 Probando diccionario: {dict_name}")
            
            dictionary = aruco.getPredefinedDictionary(dict_id)
            
            # Crear board
            try:
                board = aruco.CharucoBoard((self.col_count, self.row_count), 
                                          self.square_size, self.marker_size, dictionary)
            except:
                try:
                    board = aruco.CharucoBoard(self.col_count, self.row_count, 
                                              self.square_size, self.marker_size, dictionary)
                except:
                    board = aruco.CharucoBoard_create(self.col_count, self.row_count, 
                                                     self.square_size, self.marker_size, dictionary)
            
            total_markers = 0
            total_charuco = 0
            valid_images = 0
            
            # Probar con las primeras 5 imágenes
            for img_path in images[:5]:
                img = cv2.imread(img_path)
                if img is None:
                    continue
                    
                gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
                
                # Detectar marcadores
                corners, ids, _ = aruco.detectMarkers(gray, dictionary)
                
                # Crear imagen de visualización
                vis_img = img.copy()
                
                if ids is not None:
                    total_markers += len(ids)
                    
                    # Dibujar marcadores detectados
                    aruco.drawDetectedMarkers(vis_img, corners, ids)
                    
                    # Intentar interpolar Charuco
                    try:
                        ret, charuco_corners, charuco_ids = aruco.interpolateCornersCharuco(
                            corners, ids, gray, board
                        )
                        
                        if charuco_corners is not None and ret is not None and ret > 0:
                            total_charuco += ret
                            valid_images += 1
                            
                            # Dibujar esquinas Charuco
                            aruco.drawDetectedCornersCharuco(vis_img, charuco_corners, charuco_ids)
                            
                            cv2.putText(vis_img, f"✓ {ret} esquinas", (50, 50), 
                                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                        else:
                            cv2.putText(vis_img, "✗ Sin esquinas Charuco", (50, 50), 
                                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                    except Exception as e:
                        cv2.putText(vis_img, f"Error: {str(e)[:20]}", (50, 50), 
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
                else:
                    cv2.putText(vis_img, "✗ Sin marcadores", (50, 50), 
                               cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                
                # Añadir información
                cv2.putText(vis_img, f"Diccionario: {dict_name}", (50, 100), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                cv2.putText(vis_img, f"Imagen: {os.path.basename(img_path)}", (50, 150), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                
                # Mostrar
                cv2.imshow('Diagnóstico Charuco', vis_img)
                key = cv2.waitKey(2000) & 0xFF  # Mostrar cada imagen 2 segundos
                
                if key == 27:  # ESC para cancelar
                    cv2.destroyAllWindows()
                    return "failed"
            
            Logger.loginfo(f"   Marcadores: {total_markers}, Esquinas: {total_charuco}, Válidas: {valid_images}/5")
            
            if valid_images > best_count:
                best_count = valid_images
                best_dict = dict_name
        
        cv2.destroyAllWindows()
        
        Logger.loginfo("\n" + "="*60)
        Logger.loginfo(f"🎯 MEJOR DICCIONARIO: {best_dict} con {best_count}/5 imágenes válidas")
        Logger.loginfo("="*60)
        
        if best_count == 0:
            Logger.logerr("❌ NO SE DETECTÓ NINGÚN MARCADOR")
            Logger.loginfo("\nPOSIBLES CAUSAS:")
            Logger.loginfo("1. El tablero impreso no tiene suficiente contraste")
            Logger.loginfo("2. La iluminación es demasiado baja o hay reflejos")
            Logger.loginfo("3. El tablero está demasiado lejos (los marcadores muy pequeños)")
            Logger.loginfo("4. El diccionario ArUco es incorrecto (prueba con DICT_4X4_50 o DICT_4X4_250)")
            Logger.loginfo("5. La imagen está muy comprimida o borrosa")
            return "failed"
        
        # Continuar con la calibración normal
        Logger.loginfo(f"🚀 Iniciando calibración con {best_dict}...")
        
        try:
            # Modificar el archivo temporal con el mejor diccionario
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
            
            # Reemplazar el diccionario
            import re
            new_content = re.sub(r'dictionary: "DICT_\w+"', f'dictionary: "{dict_name}"', content)
            
            with open(self.temp_config, 'w') as f:
                f.write(new_content)
            
            Logger.loginfo(f"✅ Diccionario actualizado a {dict_name} en archivo temporal")
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
