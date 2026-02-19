#!/usr/bin/env python3
from flexbe_core import EventState, Logger
import cv2
import pyrealsense2 as rs
import numpy as np
import os
import time
from ament_index_python.packages import get_package_share_directory

class TakePictureState(EventState):
    """
    Captura imágenes usando cámara RealSense para calibración.
    
    -- pic_num          int     Número de imágenes a capturar
    -- camera_type      str     Tipo de cámara ('realsense' o 'usb')
    
    <= done                     Captura completada exitosamente
    <= failed                   Error en la captura o cancelado por usuario
    """
    
    def __init__(self, pic_num, camera_type):
        super(TakePictureState, self).__init__(outcomes=['done', 'failed'])
        
        self.pic_num = pic_num
        self.camera_type = camera_type
        self.images_saved = 1  # Comenzamos en 1 para nombres tipo 01, 02, etc.
        self.pipeline = None
        self.color_image = None
        
        # Configurar ruta de guardado
        try:
            self.save_pwd = get_package_share_directory('charuco_detector') + '/config/camera_calibration/pic/'
        except:
            # Fallback para cuando no está instalado
            self.save_pwd = os.path.expanduser('~/static/drims2_ws/install/charuco_detector/share/charuco_detector/config/camera_calibration/pic/')
        
        # Crear directorio si no existe
        os.makedirs(self.save_pwd, exist_ok=True)
        
        Logger.loginfo(f"📷 Las imágenes se guardarán en: {self.save_pwd}")
        
    def on_start(self):
        """Inicializar la cámara"""
        if self.camera_type == 'realsense':
            try:
                # Configurar streams de RealSense
                self.pipeline = rs.pipeline()
                config = rs.config()
                config.enable_stream(rs.stream.color, 1920, 1080, rs.format.bgr8, 30)
                
                # Iniciar streaming
                self.pipeline.start(config)
                Logger.loginfo("✅ RealSense iniciada correctamente")
                
                # Pequeña pausa para que la cámara se estabilice
                time.sleep(1.0)
                
                # Obtener y mostrar información de la cámara
                active_profile = self.pipeline.get_active_profile()
                color_stream = active_profile.get_stream(rs.stream.color)
                intrinsics = color_stream.as_video_stream_profile().get_intrinsics()
                Logger.loginfo(f"📷 Resolución: {intrinsics.width}x{intrinsics.height}")
                
            except Exception as e:
                Logger.logerr(f"❌ Error iniciando RealSense: {str(e)}")
                return 'failed'
        
        elif self.camera_type == 'usb':
            # Para cámara USB normal - NO USAMOS ESTO CON REAL SENSE
            Logger.logwarn("⚠️ Usando cámara USB en lugar de RealSense")
            self.capture = cv2.VideoCapture(0)
            if not self.capture.isOpened():
                Logger.logerr("❌ No se pudo abrir la cámara USB")
                return 'failed'
            Logger.loginfo("✅ Cámara USB iniciada correctamente")
    
    def execute(self, userdata):
        """Bucle principal de captura"""
        
        try:
            if self.camera_type == 'realsense':
                # Esperar por frames de RealSense
                frames = self.pipeline.wait_for_frames(timeout_ms=5000)
                color_frame = frames.get_color_frame()
                
                if not color_frame:
                    Logger.logwarn("⏳ Esperando frames de RealSense...")
                    return  # No cambiar estado aún
                
                # Convertir a numpy array
                self.color_image = np.asanyarray(color_frame.get_data())
                
            else:  # USB camera
                ret, self.color_image = self.capture.read()
                if not ret:
                    Logger.logerr("❌ Error leyendo de cámara USB")
                    return 'failed'
            
            # Mostrar la imagen con instrucciones
            display_image = self.color_image.copy()
            
            # Agregar texto de instrucciones
            cv2.putText(display_image, f"Imagen: {self.images_saved}/{self.pic_num}", 
                       (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 255, 0), 3)
            cv2.putText(display_image, "ENTER: Capturar | ESC: Salir", 
                       (50, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            
            # Mostrar ventana
            cv2.namedWindow('Captura para Calibración - RealSense', cv2.WINDOW_NORMAL)
            cv2.imshow('Captura para Calibración - RealSense', display_image)
            
            key = cv2.waitKey(1) & 0xFF
            
            if key == 13:  # ENTER - capturar imagen
                # Generar nombre de archivo con formato de 2 dígitos
                filename = f"{self.save_pwd}camera-pic-of-charucoboard-{self.images_saved:02d}.jpg"
                
                # Guardar imagen
                cv2.imwrite(filename, self.color_image)
                Logger.loginfo(f"✅ Imagen {self.images_saved}/{self.pic_num} guardada: {filename}")
                
                # Verificar que se guardó correctamente
                if os.path.exists(filename):
                    file_size = os.path.getsize(filename)
                    Logger.loginfo(f"   📦 Tamaño: {file_size} bytes")
                else:
                    Logger.logerr(f"❌ Error: No se pudo guardar {filename}")
                
                # Feedback visual
                feedback = self.color_image.copy()
                cv2.putText(feedback, f"¡Capturada {self.images_saved}/{self.pic_num}!", 
                           (50, 150), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 0), 3)
                cv2.imshow('Captura para Calibración - RealSense', feedback)
                cv2.waitKey(500)  # Mostrar confirmación por 500ms
                
                self.images_saved += 1
                
                # Verificar si ya tenemos todas las imágenes
                if self.images_saved > self.pic_num:
                    Logger.loginfo(f"🎉 Captura completada: {self.pic_num} imágenes guardadas")
                    cv2.destroyAllWindows()
                    return 'done'
                    
            elif key == 27:  # ESC - cancelar
                Logger.logwarn("⏹️ Captura cancelada por el usuario")
                cv2.destroyAllWindows()
                return 'failed'
            
        except Exception as e:
            Logger.logerr(f"❌ Error durante captura: {str(e)}")
            import traceback
            traceback.print_exc()
            return 'failed'
        
        # Continuar en el mismo estado
        return
    
    def on_stop(self):
        """Limpiar recursos al detener"""
        self._cleanup()
    
    def on_exit(self, userdata):
        """Limpiar recursos al salir"""
        self._cleanup()
    
    def _cleanup(self):
        """Liberar recursos de cámara"""
        try:
            cv2.destroyAllWindows()
            
            if self.pipeline:
                self.pipeline.stop()
                Logger.loginfo("🛑 Pipeline de RealSense detenido")
            
            if hasattr(self, 'capture') and self.capture:
                self.capture.release()
                Logger.loginfo("🛑 Cámara USB liberada")
                
        except Exception as e:
            Logger.logwarn(f"Error en limpieza: {str(e)}")
