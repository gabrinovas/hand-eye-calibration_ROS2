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
        
        # Configurar carpetas de salida
        self.output_folder = output_folder or '/home/drims/drims_ws/calibrations/extrinsic_calib_charuco_poses'
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
        
        # Config parser para INI
        self.config = configparser.ConfigParser()
        self.config.optionxform = str
        
        # Cliente de servicio (se inicializa en on_start)
        self.calib_client = None
        
        Logger.loginfo("="*60)
        Logger.loginfo("🔧 ESTADO: ComputeCalibState (UR5e)")
        Logger.loginfo("="*60)
        Logger.loginfo(f"🎯 Modo: {'Eye-in-hand' if eye_in_hand_mode else 'Eye-to-hand'}")
        Logger.loginfo(f"📁 Archivo: {self.calibration_file_name}")
        Logger.loginfo(f"📂 Carpeta: {self.calib_results_folder}")
        Logger.loginfo(f"🚀 Auto-VISP: {launch_visp}")
    
    def on_start(self):
        """Inicializar: lanzar VISP si es necesario"""
        if self.launch_visp:
            self._ensure_visp_running()
        
        # Inicializar cliente
        ProxyServiceCaller._initialize(ComputeCalibState._node)
        self.calib_client = ProxyServiceCaller({
            '/compute_effector_camera_quick': ComputeEffectorCameraQuick
        })
    
    def _ensure_visp_running(self):
        """Lanza VISP si no está corriendo"""
        if self._is_visp_running():
            Logger.loginfo("✅ VISP ya está corriendo")
            return True
        
        Logger.loginfo("🚀 Lanzando nodo VISP...")
        
        try:
            cmd = [
                'ros2', 'run', 
                'visp_hand2eye_calibration', 
                'visp_hand2eye_calibration_calibrator',
                '--ros-args',
                '-p', f'eye_in_hand:={str(self.eye_in_hand_mode).lower()}',
                '-p', 'camera_frame:=camera_color_optical_frame',
                '-p', 'marker_frame:=charuco_frame',
                '-r', '__ns:=/visp_calibration'
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
            
            for i in range(10):
                time.sleep(1)
                if self._is_visp_running():
                    Logger.loginfo(f"✅ VISP iniciado correctamente después de {i+1}s")
                    return True
            
            # Si no responde, verificar stderr
            stderr = self.visp_process.stderr.read()
            if stderr:
                Logger.logerr(f"❌ Error iniciando VISP: {stderr}")
            return False
                
        except Exception as e:
            Logger.logerr(f"❌ Error lanzando VISP: {e}")
            return False
    
    def _is_visp_running(self):
        """Verifica si VISP está corriendo"""
        try:
            # Buscar por proceso
            for proc in psutil.process_iter(['pid', 'cmdline']):
                try:
                    if proc.info['cmdline'] and 'visp_hand2eye_calibration_calibrator' in ' '.join(proc.info['cmdline']):
                        return True
                except:
                    pass
            
            # También verificar servicio
            try:
                client = self._node.create_client(
                    ComputeEffectorCameraQuick,
                    '/compute_effector_camera_quick'
                )
                if client.wait_for_service(timeout_sec=1.0):
                    return True
            except:
                pass
                
        except Exception as e:
            Logger.logwarn(f"⚠️ Error verificando VISP: {e}")
        
        return False
    
    def execute(self, userdata):
        """Ejecuta la calibración con los datos recibidos"""
        
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
            
            # Convertir a ángulos de Euler para UR (opcional)
            quat = [res.effector_camera.rotation.x,
                   res.effector_camera.rotation.y,
                   res.effector_camera.rotation.z,
                   res.effector_camera.rotation.w]
            euler = tf_transformations.euler_from_quaternion(quat)
            Logger.loginfo(f"📐 Ángulos Euler (rad):")
            Logger.loginfo(f"   roll = {euler[0]:.6f}, pitch = {euler[1]:.6f}, yaw = {euler[2]:.6f}")
            
            # Guardar resultado
            self._save_calibration(res.effector_camera, num_poses)
            
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
        inv.translation.x = T_inv[0, 3]
        inv.translation.y = T_inv[1, 3]
        inv.translation.z = T_inv[2, 3]
        
        quat_inv = tf_transformations.quaternion_from_matrix(T_inv)
        inv.rotation.x = quat_inv[0]
        inv.rotation.y = quat_inv[1]
        inv.rotation.z = quat_inv[2]
        inv.rotation.w = quat_inv[3]
        
        return inv
    
    def _save_calibration(self, transform, num_poses):
        """Guarda la calibración en formato INI y YAML"""
        
        # ===== Guardar INI =====
        ini_path = os.path.join(self.calib_results_folder, self.calibration_file_name)
        
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
        
        calib_data = {
            'calibration_date': time.strftime("%Y-%m-%d %H:%M:%S"),
            'robot_model': 'ur5e',
            'eye_in_hand': self.eye_in_hand_mode,
            'num_poses_used': num_poses,
            'transform_matrix': T.tolist(),
            'translation': {
                'x': transform.translation.x,
                'y': transform.translation.y,
                'z': transform.translation.z
            },
            'rotation_quaternion': {
                'x': transform.rotation.x,
                'y': transform.rotation.y,
                'z': transform.rotation.z,
                'w': transform.rotation.w
            },
            'rotation_euler_rad': {
                'roll': euler[0],
                'pitch': euler[1],
                'yaw': euler[2]
            },
            'rotation_euler_deg': {
                'roll': np.degrees(euler[0]),
                'pitch': np.degrees(euler[1]),
                'yaw': np.degrees(euler[2])
            }
        }
        
        with open(yaml_path, 'w') as f:
            yaml.dump(calib_data, f, default_flow_style=False, sort_keys=False)
        
        Logger.loginfo(f"💾 Calibración guardada (YAML): {yaml_path}")
        
        # También guardar una copia en la carpeta raíz de output
        root_yaml = os.path.join(self.output_folder, 'hand_eye_calibration_ur5e.yaml')
        with open(root_yaml, 'w') as f:
            yaml.dump(calib_data, f, default_flow_style=False, sort_keys=False)
        
        Logger.loginfo(f"💾 Calibración guardada (raíz): {root_yaml}")
    
    def on_stop(self):
        """Limpiar proceso de VISP al terminar"""
        if self.visp_process:
            Logger.loginfo("🛑 Deteniendo VISP...")
            try:
                # Enviar SIGTERM
                self.visp_process.terminate()
                self.visp_process.wait(timeout=3)
            except:
                try:
                    # Forzar kill si no responde
                    self.visp_process.kill()
                except:
                    pass
            self.visp_process = None
            Logger.loginfo("✅ VISP detenido")
