#!/usr/bin/env python3
from flexbe_core import EventState, Logger
import cv2
import pyrealsense2 as rs
import numpy as np
import os
import time
import glob
from ament_index_python.packages import get_package_share_directory

class TakePictureState(EventState):
    """
    Captura imágenes usando cámara RealSense para calibración.
    Las imágenes se guardan en la carpeta que usa charuco_calibrator.
    
    -- pic_num          int     Número TOTAL de imágenes a capturar (ej: 30)
    -- camera_type      str     Tipo de cámara ('realsense' o 'usb')
    -- output_folder    str     Carpeta donde guardar las imágenes (opcional)
    
    COMPORTAMIENTO:
    1. Al iniciar: BORRA automáticamente todas las imágenes existentes
    2. Tú controlas: Presiona ESPACIO o ENTER para capturar cada imagen
    3. Auto-detección: Al llegar a pic_num, cierra y pasa a calibración
    
    <= done                     Captura completada (se alcanzó pic_num)
    <= failed                   Error o cancelado por usuario (ESC)
    """
    
    def __init__(self, pic_num, camera_type, output_folder=None):
        super(TakePictureState, self).__init__(outcomes=['done', 'failed'])
        
        self.pic_num = pic_num
        self.camera_type = camera_type
        self.images_taken = 0
        self.pipeline = None
        self.color_image = None
        self.window_created = False
        self.window_name = 'CALIBRACIÓN - Toma manual de fotos'
        self.camera_initialized = False
        
        if output_folder:
            self.save_pwd = output_folder
        else:
            self.save_pwd = os.path.expanduser('~/drims_ws/calibrations/camera_calib_pictures')
        
        # Crear directorio si no existe
        os.makedirs(self.save_pwd, exist_ok=True)
        
        Logger.loginfo(f"📷 Las imágenes se guardarán en: {self.save_pwd}")
        Logger.loginfo(f"📸 Objetivo: {self.pic_num} imágenes")
        
    def on_start(self):
        """Inicializar: LIMPIAR TODO y preparar cámara"""
        
        try:
            old_images = glob.glob(os.path.join(self.save_pwd, '*.jpg'))
            if old_images:
                Logger.loginfo(f"🧹 Limpiando {len(old_images)} imágenes de ejecuciones anteriores...")
                for i, img in enumerate(old_images):
                    os.remove(img)
                Logger.loginfo("✅ Limpieza completada. Carpeta lista para nueva captura.")
            else:
                Logger.loginfo("🧹 No hay imágenes previas que limpiar. Carpeta limpia.")
        except Exception as e:
            Logger.logwarn(f"⚠️ Error durante limpieza: {str(e)}")
        
        # Inicializar cámara
        self.camera_initialized = self._init_camera()
        
        if not self.camera_initialized:
            Logger.logerr("❌ No se pudo inicializar la cámara. Verifica la conexión.")
    
    def _init_camera(self):
        """Inicializa la cámara según el tipo"""
        try:
            if self.camera_type == 'realsense':
                # Intentar con diferentes configuraciones
                try:
                    self.pipeline = rs.pipeline()
                    config = rs.config()
                    config.enable_stream(rs.stream.color, 1920, 1080, rs.format.bgr8, 30)
                    self.pipeline.start(config)
                    Logger.loginfo("✅ RealSense iniciada correctamente (1920x1080)")
                except Exception as e:
                    Logger.logwarn(f"⚠️ No se pudo iniciar con 1920x1080: {str(e)}")
                    self.pipeline = rs.pipeline()
                    config = rs.config()
                    config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
                    self.pipeline.start(config)
                    Logger.loginfo("✅ RealSense iniciada correctamente (1280x720)")
                
                time.sleep(1.0)
                active_profile = self.pipeline.get_active_profile()
                color_stream = active_profile.get_stream(rs.stream.color)
                intrinsics = color_stream.as_video_stream_profile().get_intrinsics()
                Logger.loginfo(f"📷 Resolución: {intrinsics.width}x{intrinsics.height}")
                return True
                
            elif self.camera_type == 'usb':
                Logger.loginfo("🔌 Intentando abrir cámara USB...")
                self.capture = cv2.VideoCapture(0)
                if not self.capture.isOpened():
                    Logger.logerr("❌ No se pudo abrir la cámara USB")
                    return False
                
                self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
                self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
                
                ret, frame = self.capture.read()
                if not ret:
                    Logger.logerr("❌ No se pudo leer frame de prueba")
                    return False
                
                Logger.loginfo("✅ Cámara USB iniciada correctamente")
                return True
                
        except Exception as e:
            Logger.logerr(f"❌ Error iniciando cámara: {str(e)}")
            return False
    
    def execute(self, userdata):
        """Bucle principal"""
        
        if not self.camera_initialized:
            Logger.logerr("❌ Cámara no disponible. Abortando.")
            return 'failed'
        
        try:
            # Obtener frame
            if self.camera_type == 'realsense':
                try:
                    frames = self.pipeline.wait_for_frames(timeout_ms=5000)
                    color_frame = frames.get_color_frame()
                    
                    if not color_frame:
                        Logger.logwarn("⏳ Esperando frames...")
                        return
                    
                    self.color_image = np.asanyarray(color_frame.get_data())
                    
                except RuntimeError as e:
                    Logger.logerr(f"❌ Error con RealSense: {str(e)}")
                    return 'failed'
                    
            else:  # USB
                ret, self.color_image = self.capture.read()
                if not ret:
                    Logger.logerr("❌ Error leyendo de cámara USB")
                    return 'failed'
            
            # Crear ventana
            if not self.window_created:
                cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
                cv2.resizeWindow(self.window_name, 1280, 720)
                self.window_created = True
                Logger.loginfo("🪟 Ventana de captura creada")
            
            # Preparar imagen
            display_image = self.color_image.copy()
            remaining = self.pic_num - self.images_taken
            
            # Overlay de información
            h, w = display_image.shape[:2]
            overlay = display_image.copy()
            cv2.rectangle(overlay, (10, 10), (w-10, 220), (0, 0, 0), -1)
            cv2.addWeighted(overlay, 0.5, display_image, 0.5, 0, display_image)
            
            cv2.putText(display_image, f"📸 CAPTURADAS: {self.images_taken}/{self.pic_num}", 
                       (50, 60), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 255, 0), 3)
            cv2.putText(display_image, f"⏳ FALTAN: {remaining}", 
                       (50, 120), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 255), 2)
            cv2.putText(display_image, f"📁 Guardando en: {os.path.basename(self.save_pwd)}", 
                       (50, 170), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
            cv2.putText(display_image, "👉 ESPACIO o ENTER: Tomar foto | ESC: Cancelar", 
                       (50, 220), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
            
            cv2.imshow(self.window_name, display_image)
            key = cv2.waitKey(1) & 0xFF
            
            if key == 13 or key == 32:  # ENTER o ESPACIO
                
                filename = os.path.join(self.save_pwd, f"camera-pic-of-charucoboard-{self.images_taken + 1:02d}.jpg")
                
                cv2.imwrite(filename, self.color_image)
                self.images_taken += 1
                
                Logger.loginfo(f"✅ FOTO {self.images_taken}/{self.pic_num} guardada: {os.path.basename(filename)}")
                
                # Feedback visual
                feedback = self.color_image.copy()
                overlay = feedback.copy()
                cv2.rectangle(overlay, (h//4, w//4), (3*h//4, 3*w//4), (0, 255, 0), -1)
                cv2.addWeighted(overlay, 0.3, feedback, 0.7, 0, feedback)
                cv2.putText(feedback, f"¡FOTO {self.images_taken}/{self.pic_num} GUARDADA!", 
                           (50, 300), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 0), 3)
                cv2.imshow(self.window_name, feedback)
                cv2.waitKey(500)
                
                if self.images_taken >= self.pic_num:
                    Logger.loginfo(f"🎯 ¡OBJETIVO ALCANZADO! {self.pic_num} imágenes")
                    Logger.loginfo("🔄 Pasando a calibración...")
                    cv2.destroyWindow(self.window_name)
                    self.window_created = False
                    return 'done'
                    
            elif key == 27:  # ESC
                Logger.logwarn("⏹️ Captura cancelada")
                cv2.destroyWindow(self.window_name)
                self.window_created = False
                return 'failed'
            
        except Exception as e:
            Logger.logerr(f"❌ Error: {str(e)}")
            if self.window_created:
                cv2.destroyWindow(self.window_name)
                self.window_created = False
            return 'failed'
        
        return
    
    def on_stop(self):
        self._cleanup()
    
    def on_exit(self, userdata):
        self._cleanup()
    
    def _cleanup(self):
        """Liberar recursos"""
        try:
            if self.window_created:
                try:
                    cv2.destroyWindow(self.window_name)
                    Logger.loginfo("🪟 Ventana cerrada")
                except:
                    pass
                self.window_created = False
            
            if hasattr(self, 'pipeline') and self.pipeline is not None:
                try:
                    self.pipeline.stop()
                    Logger.loginfo("🛑 RealSense detenido")
                except Exception as e:
                    Logger.logwarn(f"⚠️ Error: {str(e)}")
                finally:
                    self.pipeline = None
            
            if hasattr(self, 'capture') and self.capture is not None:
                try:
                    self.capture.release()
                    Logger.loginfo("🛑 Cámara USB liberada")
                except Exception as e:
                    Logger.logwarn(f"⚠️ Error: {str(e)}")
                finally:
                    self.capture = None
                
        except Exception as e:
            Logger.logwarn(f"Error en limpieza: {str(e)}")
