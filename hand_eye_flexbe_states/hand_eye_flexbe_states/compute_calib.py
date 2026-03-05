#!/usr/bin/env python3
"""
Estado FlexBE para calcular calibración ojo-mano con VISP.
Lanza VISP automáticamente si no está corriendo.
Adaptado para UR5e.
"""

import configparser
import yaml
import os
import subprocess
import time
import psutil
import numpy as np
import signal
from geometry_msgs.msg import Transform
from visp_hand2eye_calibration.msg import TransformArray
from visp_hand2eye_calibration.srv import ComputeEffectorCameraQuick
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyServiceCaller
import tf_transformations

class ComputeCalibState(EventState):
    """
    Calcula calibración ojo-mano con VISP (auto-lanzado).
    
    -- eye_in_hand_mode      bool     True: eye-in-hand
    -- calibration_file_name string   Nombre archivo salida
    -- customize_file        bool     Usar nombre personalizado
    -- launch_visp           bool     Lanzar VISP automáticamente
    -- output_folder         string   Carpeta de salida
    
    ># base_h_tool           TransformArray  Poses robot (base→tool)
    ># camera_h_charuco      TransformArray  Poses tablero (cámara→charuco)
    
    <= finish                Calibración completada
    <= failed                Error
    """
    
    def __init__(self, eye_in_hand_mode, calibration_file_name, 
                 customize_file=False, launch_visp=True, output_folder=None):
        super().__init__(
            outcomes=['finish', 'failed'],
            input_keys=['base_h_tool', 'camera_h_charuco']
        )
        
        self.eye_in_hand_mode = eye_in_hand_mode
        self.launch_visp = launch_visp
        self.visp_process = None
        self.calib_client = None
        self._service_ready = False
        
        # Configurar carpetas de salida
        self.output_folder = output_folder or '/home/drims/drims_ws/calibrations'
        self.calib_results_folder = os.path.join(self.output_folder, 'calibration_results')
        os.makedirs(self.calib_results_folder, exist_ok=True)
        
        # Archivo de salida
        if customize_file:
            self.calibration_file_name = str(calibration_file_name)
        else:
            if eye_in_hand_mode:
                self.calibration_file_name = "eye_in_hand_calibration_ur5e.ini"
            else:
                self.calibration_file_name = "eye_to_hand_calibration_ur5e.ini"
        
        # Archivo principal de salida
        self.main_output_file = '/home/drims/drims_ws/calibrations/camera_extrinsics.yaml'
        
        # Archivo de detecciones duplicado a eliminar
        self.temp_detections_file = '/home/drims/drims_ws/calibrations/charuco_detections.yaml'
        
        # Config parser para INI
        self.config = configparser.ConfigParser()
        self.config.optionxform = str
        
        Logger.loginfo("="*60)
        Logger.loginfo("🔧 ESTADO: ComputeCalibState (UR5e)")
        Logger.loginfo("="*60)
        Logger.loginfo(f"🎯 Modo: {'Eye-in-hand' if eye_in_hand_mode else 'Eye-to-hand'}")
        Logger.loginfo(f"📁 Archivo: {self.calibration_file_name}")
        Logger.loginfo(f"📂 Carpeta: {self.calib_results_folder}")
        Logger.loginfo(f"📄 Archivo principal: {self.main_output_file}")
        Logger.loginfo(f"📄 Archivo temp a eliminar: {self.temp_detections_file}")
        Logger.loginfo(f"🚀 Auto-VISP: {launch_visp}")
    
    def on_start(self):
        """Inicializar: lanzar VISP si es necesario"""
        # Inicializar el proxy con el nodo de FlexBE
        ProxyServiceCaller.initialize(ComputeCalibState._node)
        
        if self.launch_visp:
            self._ensure_visp_running()
        
        # Ahora crear el cliente
        self.calib_client = ProxyServiceCaller({
            '/compute_effector_camera_quick': ComputeEffectorCameraQuick
        })
        
        # Verificar que el servicio está disponible
        if self.calib_client.is_available('/compute_effector_camera_quick'):
            self._service_ready = True
            Logger.loginfo("✅ Servicio VISP disponible")
        else:
            Logger.logwarn("⏳ Esperando servicio VISP...")
    
    def _ensure_visp_running(self):
        """Lanza VISP si no está corriendo"""
        if self._is_visp_running():
            Logger.loginfo("✅ VISP ya está corriendo")
            return True
        
        Logger.loginfo("🚀 Lanzando nodo VISP...")
        
        try:
            # Convertir booleano a string 'true'/'false' para ROS2
            eye_in_hand_str = 'true' if self.eye_in_hand_mode else 'false'
            
            cmd = [
                'ros2', 'run', 
                'visp_hand2eye_calibration', 
                'visp_hand2eye_calibration_calibrator',
                '--ros-args',
                '-p', f'eye_in_hand:={eye_in_hand_str}',
                '-p', 'camera_frame:=camera_color_optical_frame',
                '-p', 'marker_frame:=charuco_frame'
            ]
            
            Logger.loginfo(f"📋 Comando: {' '.join(cmd)}")
            
            self.visp_process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                start_new_session=True,
                text=True
            )
            
            # Esperar a que inicie
            Logger.loginfo("⏳ Esperando a que VISP inicie...")
            
            for i in range(15):
                time.sleep(1)
                if self._is_visp_running():
                    Logger.loginfo(f"✅ VISP iniciado correctamente después de {i+1}s")
                    return True
                
                # Verificar si el proceso falló
                if self.visp_process.poll() is not None:
                    stdout, stderr = self.visp_process.communicate()
                    Logger.logerr(f"❌ VISP terminó prematuramente")
                    if stderr:
                        Logger.logerr(f"Error: {stderr}")
                    return False
            
            Logger.logwarn("⚠️ VISP no responde pero podría estar iniciando...")
            return True
            
        except Exception as e:
            Logger.logerr(f"❌ Error lanzando VISP: {e}")
            return False
    
    def _is_visp_running(self):
        """Verifica si VISP está corriendo"""
        try:
            # Buscar por proceso
            for proc in psutil.process_iter(['pid', 'cmdline']):
                try:
                    if proc.info['cmdline'] and any('visp_hand2eye_calibration_calibrator' in cmd for cmd in proc.info['cmdline'] if cmd):
                        return True
                except:
                    pass
            
            # También verificar servicio
            try:
                result = subprocess.run(
                    ['ros2', 'service', 'list'],
                    capture_output=True,
                    text=True,
                    timeout=2
                )
                if '/compute_effector_camera_quick' in result.stdout:
                    return True
            except:
                pass
                
        except Exception as e:
            Logger.logwarn(f"⚠️ Error verificando VISP: {e}")
        
        return False
    
    def execute(self, userdata):
        """Ejecuta la calibración con los datos recibidos"""
        
        # Verificar que el servicio está listo
        if not self._service_ready:
            if self.calib_client and self.calib_client.is_available('/compute_effector_camera_quick'):
                self._service_ready = True
                Logger.loginfo("✅ Servicio VISP ahora disponible")
            else:
                Logger.logwarn("⏳ Esperando a que el servicio VISP esté disponible...")
                return None  # No fallar, solo esperar
        
        # Verificar datos
        if not hasattr(userdata.base_h_tool, 'transforms') or len(userdata.base_h_tool.transforms) == 0:
            Logger.logerr("❌ No se recibieron poses del robot")
            return 'failed'
        
        if not hasattr(userdata.camera_h_charuco, 'transforms') or len(userdata.camera_h_charuco.transforms) == 0:
            Logger.logerr("❌ No se recibieron poses del tablero")
            return 'failed'
        
        # Verificar que coincidan número de poses
        if len(userdata.base_h_tool.transforms) != len(userdata.camera_h_charuco.transforms):
            Logger.logerr(f"❌ Número de poses no coincide: robot={len(userdata.base_h_tool.transforms)}, tablero={len(userdata.camera_h_charuco.transforms)}")
            return 'failed'
        
        num_poses = len(userdata.base_h_tool.transforms)
        Logger.loginfo(f"📊 Calibrando con {num_poses} poses")
        
        # Verificar servicio disponible
        if not self.calib_client.is_available('/compute_effector_camera_quick'):
            Logger.logerr("❌ Servicio de VISP no disponible")
            return 'failed'
        
        # Preparar request
        req = ComputeEffectorCameraQuick.Request()
        req.camera_object = TransformArray()
        req.world_effector = TransformArray()
        req.camera_object.header = userdata.camera_h_charuco.header
        req.world_effector.header = userdata.base_h_tool.header
        
        # Procesar según modo
        if self.eye_in_hand_mode:
            # Modo eye-in-hand: usar directamente
            Logger.loginfo("🔄 Modo Eye-in-hand: usando poses directamente")
            req.world_effector.transforms = userdata.base_h_tool.transforms
            req.camera_object.transforms = userdata.camera_h_charuco.transforms
        else:
            # Modo eye-to-hand: invertir transforms
            Logger.loginfo("🔄 Modo Eye-to-hand: invirtiendo transforms")
            
            for t in userdata.base_h_tool.transforms:
                inv = self._invert_transform(t)
                req.world_effector.transforms.append(inv)
            
            for t in userdata.camera_h_charuco.transforms:
                inv = self._invert_transform(t)
                req.camera_object.transforms.append(inv)
        
        try:
            # Llamar al servicio
            Logger.loginfo("📡 Llamando a servicio de calibración VISP...")
            res = self.calib_client.call('/compute_effector_camera_quick', req)
            
            if res is None:
                Logger.logerr("❌ El servicio no devolvió respuesta")
                return 'failed'
            
            # Mostrar resultado
            Logger.loginfo("="*60)
            Logger.loginfo("✅ CALIBRACIÓN COMPLETADA - UR5e")
            Logger.loginfo("="*60)
            Logger.loginfo(f"📐 Traslación (metros):")
            Logger.loginfo(f"   x = {res.effector_camera.translation.x:.6f}")
            Logger.loginfo(f"   y = {res.effector_camera.translation.y:.6f}")
            Logger.loginfo(f"   z = {res.effector_camera.translation.z:.6f}")
            Logger.loginfo(f"🌀 Cuaternión:")
            Logger.loginfo(f"   qx = {res.effector_camera.rotation.x:.6f}")
            Logger.loginfo(f"   qy = {res.effector_camera.rotation.y:.6f}")
            Logger.loginfo(f"   qz = {res.effector_camera.rotation.z:.6f}")
            Logger.loginfo(f"   qw = {res.effector_camera.rotation.w:.6f}")
            
            # Convertir a ángulos de Euler para UR
            quat = [res.effector_camera.rotation.x,
                   res.effector_camera.rotation.y,
                   res.effector_camera.rotation.z,
                   res.effector_camera.rotation.w]
            euler = tf_transformations.euler_from_quaternion(quat)
            Logger.loginfo(f"📐 Ángulos Euler (rad):")
            Logger.loginfo(f"   roll = {euler[0]:.6f}, pitch = {euler[1]:.6f}, yaw = {euler[2]:.6f}")
            
            # Guardar resultado
            self._save_calibration(res.effector_camera, num_poses)
            
            # Eliminar archivo temporal de detecciones
            self._cleanup_temp_files()
            
            return 'finish'
            
        except Exception as e:
            Logger.logerr(f"❌ Error en calibración: {e}")
            return 'failed'
    
    def _invert_transform(self, t):
        """Invierte una transformación"""
        # Matriz homogénea
        T = np.eye(4)
        quat = [t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w]
        R = tf_transformations.quaternion_matrix(quat)[:3, :3]
        T[:3, :3] = R
        T[:3, 3] = [t.translation.x, t.translation.y, t.translation.z]
        
        # Invertir
        T_inv = np.linalg.inv(T)
        
        # Crear transform inverso
        inv = Transform()
        inv.translation.x = float(T_inv[0, 3])
        inv.translation.y = float(T_inv[1, 3])
        inv.translation.z = float(T_inv[2, 3])
        
        quat_inv = tf_transformations.quaternion_from_matrix(T_inv)
        inv.rotation.x = float(quat_inv[0])
        inv.rotation.y = float(quat_inv[1])
        inv.rotation.z = float(quat_inv[2])
        inv.rotation.w = float(quat_inv[3])
        
        return inv
    
    def _cleanup_temp_files(self):
        """Elimina archivos temporales después de la calibración"""
        try:
            if os.path.exists(self.temp_detections_file):
                os.remove(self.temp_detections_file)
                Logger.loginfo(f"🧹 Archivo temporal eliminado: {self.temp_detections_file}")
            else:
                Logger.loginfo(f"📂 Archivo temporal no encontrado (ya eliminado): {self.temp_detections_file}")
        except Exception as e:
            Logger.logwarn(f"⚠️ No se pudo eliminar el archivo temporal {self.temp_detections_file}: {e}")
    
    def _save_calibration(self, transform, num_poses):
        """Guarda la calibración en formato INI, YAML y archivo principal"""
        
        # ===== Guardar INI =====
        ini_filename = self.calibration_file_name
        if not ini_filename.endswith('.ini'):
            ini_filename += '.ini'
        
        ini_path = os.path.join(self.calib_results_folder, ini_filename)
        
        if os.path.exists(ini_path):
            self.config.read(ini_path)
        
        if not self.config.has_section("hand_eye_calibration"):
            self.config.add_section("hand_eye_calibration")
        
        self.config.set("hand_eye_calibration", "x", str(transform.translation.x))
        self.config.set("hand_eye_calibration", "y", str(transform.translation.y))
        self.config.set("hand_eye_calibration", "z", str(transform.translation.z))
        self.config.set("hand_eye_calibration", "qx", str(transform.rotation.x))
        self.config.set("hand_eye_calibration", "qy", str(transform.rotation.y))
        self.config.set("hand_eye_calibration", "qz", str(transform.rotation.z))
        self.config.set("hand_eye_calibration", "qw", str(transform.rotation.w))
        self.config.set("hand_eye_calibration", "num_poses_used", str(num_poses))
        self.config.set("hand_eye_calibration", "eye_in_hand", str(self.eye_in_hand_mode))
        self.config.set("hand_eye_calibration", "robot_model", "ur5e")
        self.config.set("hand_eye_calibration", "timestamp", str(time.time()))
        
        with open(ini_path, 'w') as f:
            self.config.write(f)
        
        Logger.loginfo(f"💾 Calibración guardada (INI): {ini_path}")
        
        # ===== Guardar YAML =====
        yaml_path = ini_path.replace('.ini', '.yaml')
        
        # Matriz de transformación completa
        T = np.eye(4)
        quat = [transform.rotation.x, transform.rotation.y, 
                transform.rotation.z, transform.rotation.w]
        R = tf_transformations.quaternion_matrix(quat)[:3, :3]
        T[:3, :3] = R
        T[:3, 3] = [transform.translation.x, transform.translation.y, transform.translation.z]
        
        # Convertir a ángulos de Euler para UR
        euler = tf_transformations.euler_from_quaternion(quat)
        
        # Convertir matriz a lista de floats nativos
        transform_matrix = []
        for row in T.tolist():
            transform_matrix.append([float(x) for x in row])
        
        # Convertir todos los valores numpy a floats nativos
        calib_data = {
            'calibration_date': time.strftime("%Y-%m-%d %H:%M:%S"),
            'robot_model': 'ur5e',
            'eye_in_hand': bool(self.eye_in_hand_mode),
            'num_poses_used': int(num_poses),
            'transform_matrix': transform_matrix,
            'translation': {
                'x': float(transform.translation.x),
                'y': float(transform.translation.y),
                'z': float(transform.translation.z)
            },
            'rotation_quaternion': {
                'x': float(transform.rotation.x),
                'y': float(transform.rotation.y),
                'z': float(transform.rotation.z),
                'w': float(transform.rotation.w)
            },
            'rotation_euler_rad': {
                'roll': float(euler[0]),
                'pitch': float(euler[1]),
                'yaw': float(euler[2])
            },
            'rotation_euler_deg': {
                'roll': float(np.degrees(euler[0])),
                'pitch': float(np.degrees(euler[1])),
                'yaw': float(np.degrees(euler[2]))
            }
        }
        
        with open(yaml_path, 'w') as f:
            yaml.dump(calib_data, f, default_flow_style=False, sort_keys=False, indent=2)
        
        Logger.loginfo(f"💾 Calibración guardada (YAML): {yaml_path}")
        
        # ===== Guardar archivo principal =====
        with open(self.main_output_file, 'w') as f:
            yaml.dump(calib_data, f, default_flow_style=False, sort_keys=False, indent=2)
        
        Logger.loginfo(f"💾 Calibración guardada en archivo principal: {self.main_output_file}")
        
        # ===== Guardar extrinsic_matrix.yaml con formato específico =====
        extrinsic_file = os.path.join(self.output_folder, 'extrinsic_matrix.yaml')
        
        # Calcular T_c2w (inversa de T)
        T_c2w = np.linalg.inv(T)
        
        # Convertir matrices a listas planas de 16 elementos
        T_w2c_flat = T.flatten().tolist()
        T_c2w_flat = T_c2w.flatten().tolist()
        
        # Convertir todos los valores a float nativo con alta precisión
        T_w2c_flat = [float(x) for x in T_w2c_flat]
        T_c2w_flat = [float(x) for x in T_c2w_flat]
        
        # Crear estructura con clave 'camera' e índice 1
        extrinsic_data = {
            'camera': {
                1: {
                    'T_c2w': T_c2w_flat,
                    'T_w2c': T_w2c_flat
                }
            }
        }
        
        with open(extrinsic_file, 'w') as f:
            # Usar default_flow_style=None para formato legible pero compacto
            yaml.dump(extrinsic_data, f, default_flow_style=None, sort_keys=False, indent=2)
        
        Logger.loginfo(f"💾 Matriz extrínseca guardada en: {extrinsic_file}")
    
    def on_stop(self):
        """Limpiar proceso de VISP al terminar"""
        if self.visp_process:
            Logger.loginfo("🛑 Deteniendo VISP...")
            try:
                os.killpg(os.getpgid(self.visp_process.pid), signal.SIGINT)
                self.visp_process.wait(timeout=5)
            except:
                try:
                    self.visp_process.terminate()
                    self.visp_process.wait(timeout=3)
                except:
                    try:
                        self.visp_process.kill()
                    except:
                        pass
            self.visp_process = None
            Logger.loginfo("✅ VISP detenido")
