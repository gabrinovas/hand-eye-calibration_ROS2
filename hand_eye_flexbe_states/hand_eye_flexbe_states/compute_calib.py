#!/usr/bin/env python3
import configparser
import os
import subprocess
import time
import psutil
import numpy as np
from geometry_msgs.msg import Transform
from visp_hand2eye_calibration.msg import TransformArray
from visp_hand2eye_calibration.srv import ComputeEffectorCameraQuick
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyServiceCaller
from ament_index_python.packages import get_package_share_directory
import tf_transformations

class ComputeCalibState(EventState):
    """
    Calcula calibración ojo-mano con VISP (auto-lanzado).
    
    -- eye_in_hand_mode      bool     True: eye-in-hand
    -- calibration_file_name string   Nombre archivo salida
    -- customize_file        bool     Usar nombre personalizado
    -- launch_visp           bool     Lanzar VISP automáticamente
    
    ># base_h_tool           TransformArray  Poses robot
    ># camera_h_charuco      TransformArray  Poses tablero
    
    <= finish                Calibración completada
    <= failed                Error
    """
    
    def __init__(self, eye_in_hand_mode, calibration_file_name, 
                 customize_file=False, launch_visp=True):
        super().__init__(
            outcomes=['finish', 'failed'],
            input_keys=['base_h_tool', 'camera_h_charuco']
        )
        
        self.eye_in_hand_mode = eye_in_hand_mode
        self.launch_visp = launch_visp
        self.visp_process = None
        
        # Archivo salida
        if customize_file:
            self.calibration_file_name = str(calibration_file_name)
        else:
            self.calibration_file_name = ("eye_in_hand_calibration.ini" if eye_in_hand_mode 
                                          else "eye_to_hand_calibration.ini")
        
        # Ruta
        self.save_pwd = os.path.join(
            get_package_share_directory('charuco_detector'),
            'config',
            'hand_eye_calibration'
        )
        os.makedirs(self.save_pwd, exist_ok=True)
        
        self.config = configparser.ConfigParser()
        self.config.optionxform = str
        
        Logger.loginfo("="*60)
        Logger.loginfo("🔧 ComputeCalibState")
        Logger.loginfo(f"Modo: {'Eye-in-hand' if eye_in_hand_mode else 'Eye-to-hand'}")
        Logger.loginfo(f"Archivo: {self.calibration_file_name}")
        Logger.loginfo(f"Auto-VISP: {launch_visp}")
    
    def on_start(self):
        if self.launch_visp:
            self._ensure_visp_running()
        
        ProxyServiceCaller._initialize(ComputeCalibState._node)
        self.calib_client = ProxyServiceCaller({
            '/compute_effector_camera_quick': ComputeEffectorCameraQuick
        })
    
    def _ensure_visp_running(self):
        """Lanza VISP si no está corriendo"""
        if self._is_visp_running():
            Logger.loginfo("✅ VISP ya está corriendo")
            return
        
        Logger.loginfo("🚀 Lanzando VISP...")
        cmd = [
            'ros2', 'run', 
            'visp_hand2eye_calibration', 
            'visp_hand2eye_calibration_calibrator',
            '--ros-args',
            '-p', f'eye_in_hand:={str(self.eye_in_hand_mode).lower()}'
        ]
        
        self.visp_process = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            start_new_session=True
        )
        
        time.sleep(3)
        if self._is_visp_running():
            Logger.loginfo("✅ VISP iniciado")
        else:
            Logger.logerr("❌ Error iniciando VISP")
    
    def _is_visp_running(self):
        """Verifica si VISP está corriendo"""
        try:
            for proc in psutil.process_iter(['pid', 'cmdline']):
                try:
                    if 'visp_hand2eye_calibration_calibrator' in ' '.join(proc.info['cmdline'] or []):
                        return True
                except:
                    pass
        except:
            pass
        return False
    
    def execute(self, userdata):
        # Verificar datos
        if not hasattr(userdata.base_h_tool, 'transforms') or len(userdata.base_h_tool.transforms) == 0:
            Logger.logerr("❌ No hay poses del robot")
            return 'failed'
        
        num_poses = len(userdata.base_h_tool.transforms)
        Logger.loginfo(f"📊 Calibrando con {num_poses} poses")
        
        # Preparar request
        req = ComputeEffectorCameraQuick.Request()
        req.camera_object = TransformArray()
        req.world_effector = TransformArray()
        req.camera_object.header = userdata.camera_h_charuco.header
        req.world_effector.header = userdata.base_h_tool.header
        
        if self.eye_in_hand_mode:
            req.world_effector.transforms = userdata.base_h_tool.transforms
            req.camera_object.transforms = userdata.camera_h_charuco.transforms
        else:
            # Invertir para eye-to-hand
            for t in userdata.base_h_tool.transforms:
                req.world_effector.transforms.append(self._invert_transform(t))
            for t in userdata.camera_h_charuco.transforms:
                req.camera_object.transforms.append(self._invert_transform(t))
        
        try:
            res = self.calib_client.call('/compute_effector_camera_quick', req)
            if not res:
                return 'failed'
            
            # Mostrar resultado
            Logger.loginfo("✅ CALIBRACIÓN COMPLETADA")
            Logger.loginfo(f"x={res.effector_camera.translation.x:.6f}")
            Logger.loginfo(f"y={res.effector_camera.translation.y:.6f}")
            Logger.loginfo(f"z={res.effector_camera.translation.z:.6f}")
            Logger.loginfo(f"qx={res.effector_camera.rotation.x:.6f}")
            Logger.loginfo(f"qy={res.effector_camera.rotation.y:.6f}")
            Logger.loginfo(f"qz={res.effector_camera.rotation.z:.6f}")
            Logger.loginfo(f"qw={res.effector_camera.rotation.w:.6f}")
            
            self._save_calibration(res.effector_camera, num_poses)
            return 'finish'
            
        except Exception as e:
            Logger.logerr(f"❌ Error: {e}")
            return 'failed'
    
    def _invert_transform(self, t):
        """Invierte una transformación"""
        T = np.eye(4)
        quat = [t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w]
        R = tf_transformations.quaternion_matrix(quat)[:3, :3]
        T[:3, :3] = R
        T[:3, 3] = [t.translation.x, t.translation.y, t.translation.z]
        T_inv = np.linalg.inv(T)
        
        inv = Transform()
        inv.translation.x = T_inv[0, 3]
        inv.translation.y = T_inv[1, 3]
        inv.translation.z = T_inv[2, 3]
        quat_inv = tf_transformations.quaternion_from_matrix(T_inv)
        inv.rotation.x, inv.rotation.y, inv.rotation.z, inv.rotation.w = quat_inv
        return inv
    
    def _save_calibration(self, transform, num_poses):
        """Guarda calibración"""
        filepath = os.path.join(self.save_pwd, self.calibration_file_name)
        
        if os.path.exists(filepath):
            self.config.read(filepath)
        
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
        
        with open(filepath, 'w') as f:
            self.config.write(f)
        
        Logger.loginfo(f"💾 Guardado en: {filepath}")
    
    def on_stop(self):
        if self.visp_process:
            self.visp_process.terminate()
