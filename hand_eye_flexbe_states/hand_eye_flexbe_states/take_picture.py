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
    
    -- pic_num          int     Número TOTAL de imágenes a capturar (ej: 30)
    -- camera_type      str     Tipo de cámara ('realsense' o 'usb')
    
    COMPORTAMIENTO:
    1. Al iniciar: BORRA automáticamente todas las imágenes existentes
    2. Tú controlas: Presiona ENTER para capturar cada imagen
    3. Auto-detección: Al llegar a pic_num, cierra y pasa a calibración
    
    <= done                     Captura completada (se alcanzó pic_num)
    <= failed                   Error o cancelado por usuario (ESC)
    """
    
    def __init__(self, pic_num, camera_type):
        super(TakePictureState, self).__init__(outcomes=['done', 'failed'])
        
        self.pic_num = pic_num
        self.camera_type = camera_type
        self.images_taken = 0  # Contador de fotos tomadas en ESTA ejecución
        self.pipeline = None
        self.color_image = None
        self.window_created = False  # Control para crear ventana SOLO UNA VEZ
        self.window_name = 'CALIBRACIÓN - Toma manual de fotos'
        
        # Configurar ruta de guardado
        try:
            self.save_pwd = get_package_share_directory('charuco_detector') + '/config/camera_calibration/pic/'
        except:
            self.save_pwd = os.path.expanduser('~/static/drims2_ws/install/charuco_detector/share/charuco_detector/config/camera_calibration/pic/')
        
        # Crear directorio si no existe
        os.makedirs(self.save_pwd, exist_ok=True)
        
        Logger.loginfo(f"📷 Las imágenes se guardarán en: {self.save_pwd}")
        Logger.loginfo(f"📸 Objetivo: {self.pic_num} imágenes")
        
    def on_start(self):
        """Inicializar: LIMPIAR TODO y preparar cámara"""
        
        # ===== 1. LIMPIEZA AUTOMÁTICA: BORRAR TODAS LAS FOTOS PREVIAS =====
        try:
            old_images = glob.glob(self.save_pwd + 'camera-pic-of-charucoboard-*.jpg')
            if old_images:
                Logger.loginfo(f"🧹 Limpiando {len(old_images)} imágenes de ejecuciones anteriores...")
                for i, img in enumerate(old_images):
                    os.remove(img)
                    if i < 5:  # Mostrar solo las primeras 5 para no saturar
                        Logger.loginfo(f"   Eliminada: {os.path.basename(img)}")
                if len(old_images) > 5:
                    Logger.loginfo(f"   ... y {len(old_images)-5} más")
                Logger.loginfo("✅ Limpieza completada. Carpeta lista para nueva captura.")
            else:
                Logger.loginfo("🧹 No hay imágenes previas que limpiar. Carpeta limpia.")
        except Exception as e:
            Logger.logwarn(f"⚠️ Error durante limpieza: {str(e)}")
        # ===================================================================
        
        # Inicializar cámara
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
            Logger.logwarn("⚠️ Usando cámara USB en lugar de RealSense")
            self.capture = cv2.VideoCapture(0)
            if not self.capture.isOpened():
                Logger.logerr("❌ No se pudo abrir la cámara USB")
                return 'failed'
            Logger.loginfo("✅ Cámara USB iniciada correctamente")
    
    def execute(self, userdata):
        """Bucle principal - TÚ controlas con ENTER"""
        
        try:
            # Obtener frame de la cámara
            if self.camera_type == 'realsense':
                frames = self.pipeline.wait_for_frames(timeout_ms=5000)
                color_frame = frames.get_color_frame()
                
                if not color_frame:
                    Logger.logwarn("⏳ Esperando frames de RealSense...")
                    return
                
                self.color_image = np.asanyarray(color_frame.get_data())
                
            else:  # USB
                ret, self.color_image = self.capture.read()
                if not ret:
                    Logger.logerr("❌ Error leyendo de cámara USB")
                    return 'failed'
            
            # ===== CREAR VENTANA SOLO UNA VEZ =====
            if not self.window_created:
                cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
                self.window_created = True
                Logger.loginfo("🪟 Ventana de captura creada")
            # ======================================
            
            # Preparar imagen para mostrar (con overlay de información)
            display_image = self.color_image.copy()
            
            # Calcular cuántas faltan
            remaining = self.pic_num - self.images_taken
            
            # Agregar información en la imagen
            cv2.putText(display_image, f"📸 CAPTURADAS: {self.images_taken}/{self.pic_num}", 
                       (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 255, 0), 3)
            cv2.putText(display_image, f"⏳ FALTAN: {remaining}", 
                       (50, 100), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 0), 2)
            cv2.putText(display_image, "👉 ENTER: Tomar foto | ESC: Cancelar", 
                       (50, 150), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            
            # Actualizar la imagen en la ventana existente (NO crear nueva)
            cv2.imshow(self.window_name, display_image)
            
            key = cv2.waitKey(1) & 0xFF
            
            if key == 13:  # ENTER - Tú decides tomar foto
                
                # Generar nombre secuencial (siempre desde 1 porque limpiamos al inicio)
                filename = f"{self.save_pwd}camera-pic-of-charucoboard-{self.images_taken + 1:02d}.jpg"
                
                # Guardar imagen
                cv2.imwrite(filename, self.color_image)
                self.images_taken += 1
                
                Logger.loginfo(f"✅ FOTO {self.images_taken}/{self.pic_num} guardada: {os.path.basename(filename)}")
                
                # Feedback visual rápido (usando la MISMA ventana)
                feedback = self.color_image.copy()
                cv2.putText(feedback, f"¡FOTO {self.images_taken}/{self.pic_num} GUARDADA!", 
                           (50, 200), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 0), 3)
                cv2.imshow(self.window_name, feedback)
                cv2.waitKey(500)  # Mostrar confirmación por medio segundo
                
                # ===== AUTO-DETECCIÓN: ¿Ya completamos? =====
                if self.images_taken >= self.pic_num:
                    Logger.loginfo(f"🎯 ¡OBJETIVO ALCANZADO! {self.pic_num} imágenes capturadas")
                    Logger.loginfo("🔄 Pasando automáticamente a fase de calibración...")
                    cv2.destroyWindow(self.window_name)  # Destruir SOLO esta ventana
                    self.window_created = False
                    return 'done'  # <-- Automático: pasa a calibración
                # ============================================
                    
            elif key == 27:  # ESC - Cancelar manualmente
                Logger.logwarn("⏹️ Captura cancelada por el usuario")
                cv2.destroyWindow(self.window_name)
                self.window_created = False
                return 'failed'
            
        except Exception as e:
            Logger.logerr(f"❌ Error durante captura: {str(e)}")
            import traceback
            traceback.print_exc()
            if self.window_created:
                cv2.destroyWindow(self.window_name)
                self.window_created = False
            return 'failed'
        
        # Seguimos en el mismo estado esperando más fotos
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
            if self.window_created:
                cv2.destroyWindow(self.window_name)
                self.window_created = False
                Logger.loginfo("🪟 Ventana de captura cerrada")
            
            if self.pipeline:
                self.pipeline.stop()
                Logger.loginfo("🛑 Pipeline de RealSense detenido")
            
            if hasattr(self, 'capture') and self.capture:
                self.capture.release()
                Logger.loginfo("🛑 Cámara USB liberada")
                
        except Exception as e:
            Logger.logwarn(f"Error en limpieza: {str(e)}")
