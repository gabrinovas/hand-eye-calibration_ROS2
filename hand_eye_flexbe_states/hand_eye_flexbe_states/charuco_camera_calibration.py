#!/usr/bin/env python3
from flexbe_core import EventState, Logger
import os
import subprocess
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
            # Por defecto, usar la carpeta del paquete charuco_calibrator
            # try:
            #     charuco_share = get_package_share_directory('charuco_calibrator')
            #     self.images_folder = os.path.join(charuco_share, 'config', 'camera_calibration')
            # except:
            #     # Fallback
            self.images_folder = os.path.expanduser('~/drims_ws/calibrations/camera_calib_pictures')
        
        # Carpeta específica de imágenes
        self.pic_folder = os.path.join(self.images_folder, 'pic')
        
        # Ruta al archivo de configuración de charuco_calibrator
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
        
        # Crear archivo temporal de configuración con los parámetros actualizados
        self.temp_config = os.path.join(self.images_folder, 'temp_charuco_params.yaml')
        self._create_temp_config()

    def _create_temp_config(self):
        """Crea un archivo de configuración temporal con los parámetros actuales"""
        config_content = f"""charuco_calibrator:
  ros__parameters:
    charuco_rows: {self.row_count}
    charuco_cols: {self.col_count}
    square_length: {self.square_size}
    marker_length: {self.marker_size}
    dictionary: "DICT_4X4_1000"
    output_file: "{self.save_file_name}"
    image_width: 1920
    image_height: 1080
    min_markers: 4
    min_corners: 10
    min_valid_images: 5
"""
        try:
            with open(self.temp_config, 'w') as f:
                f.write(config_content)
            Logger.loginfo(f"✅ Archivo de configuración temporal creado: {self.temp_config}")
        except Exception as e:
            Logger.logwarn(f"⚠️ No se pudo crear archivo temporal: {str(e)}")
    
    def on_start(self):
        pass
    
    def execute(self, userdata):
        # Verificar que existen imágenes
        import glob
        images = glob.glob(os.path.join(self.pic_folder, '*.jpg'))
        
        if not images:
            Logger.logerr(f"❌ No se encontraron imágenes en: {self.pic_folder}")
            Logger.loginfo("💡 Asegúrate de haber capturado imágenes primero con el estado 'take_picture'")
            return "failed"
        
        Logger.loginfo(f"🔍 Encontradas {len(images)} imágenes para calibración")
        Logger.loginfo(f"🚀 Iniciando calibración con charuco_calibrator...")
        
        try:
            # Construir comando para lanzar la calibración
            launch_cmd = [
                'ros2', 'launch', 'charuco_calibrator', 'charuco_detector.launch.py',
                f'images_folder:={self.images_folder}',
                f'config_file:={self.temp_config}',
                f'output_file:={self.save_file_name}',
                'show_preview:=false'
            ]
            
            Logger.loginfo(f"📋 Ejecutando: {' '.join(launch_cmd)}")
            
            # Ejecutar el launch file
            process = subprocess.Popen(
                launch_cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                universal_newlines=True,
                bufsize=1
            )
            
            # Mostrar salida en tiempo real
            output_file = None
            for line in process.stdout:
                Logger.loginfo(f"[charuco_calibrator] {line.strip()}")
                
                # Detectar si se guardó el archivo
                if "💾 Calibración guardada en:" in line:
                    parts = line.strip().split("💾 Calibración guardada en: ")
                    if len(parts) > 1:
                        output_file = parts[1]
            
            # Esperar a que termine
            process.wait()
            
            if process.returncode == 0:
                Logger.loginfo("✅ Calibración completada exitosamente")
                if output_file:
                    Logger.loginfo(f"📁 Resultado guardado en: {output_file}")
                return "done"
            else:
                Logger.logerr(f"❌ Calibración fallida con código: {process.returncode}")
                return "failed"
                
        except Exception as e:
            Logger.logerr(f"❌ Error durante calibración: {str(e)}")
            import traceback
            traceback.print_exc()
            return "failed"
            
    def on_exit(self, userdata):
        """Limpiar archivos temporales al salir"""
        try:
            if os.path.exists(self.temp_config):
                os.remove(self.temp_config)
                Logger.loginfo("🧹 Archivo de configuración temporal eliminado")
        except Exception as e:
            Logger.logwarn(f"⚠️ Error eliminando archivo temporal: {str(e)}")
