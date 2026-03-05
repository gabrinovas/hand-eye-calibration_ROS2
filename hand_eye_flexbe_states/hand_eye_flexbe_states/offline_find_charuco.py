#!/usr/bin/env python3
"""
Estado FlexBE para procesar offline las poses de Charuco.
Inicia el nodo charuco_hand_eye y espera a que publique.
"""

from flexbe_core import EventState, Logger
import os
import yaml
import subprocess
import time
from visp_hand2eye_calibration.msg import TransformArray
from std_msgs.msg import Header
import psutil
import signal
import numpy as np
import tf_transformations

class OfflineFindCharucoState(EventState):
    """
    Inicia el nodo de detección de Charuco y espera a que publique.
    """
    
    def __init__(self, pictures_folder, robot_poses_folder, output_folder=None, eye_in_hand=False):
        super().__init__(
            outcomes=['completed', 'failed'],
            output_keys=['base_h_tool_accumulated', 'camera_h_charuco_accumulated']
        )
        
        self.pictures_folder = pictures_folder
        self.robot_poses_folder = robot_poses_folder
        self.output_folder = output_folder or '/home/drims/drims_ws/calibrations/extrinsic_calibration/charuco_table_poses'
        self.eye_in_hand = eye_in_hand
        
        self.charuco_process = None
        self.base_h_tool_accumulated = None
        self.camera_h_charuco_accumulated = None
        self.received_data = False
        self.max_wait_time = 120  # segundos máximo de espera
        self.start_time = None
        
        # Suscripciones
        self.world_effector_sub = None
        self.camera_object_sub = None
        
        Logger.loginfo("="*60)
        Logger.loginfo("🔍 ESTADO: OfflineFindCharuco")
        Logger.loginfo("="*60)
        Logger.loginfo(f"📁 Imágenes: {self.pictures_folder}")
        Logger.loginfo(f"📁 Poses robot: {self.robot_poses_folder}")
        Logger.loginfo(f"📁 Salida: {self.output_folder}")
        Logger.loginfo(f"🎯 Modo: {'Eye-in-hand' if eye_in_hand else 'Eye-to-hand'}")
        
    def on_enter(self, userdata):
        """Iniciar nodo de detección y suscribirse"""
        self.received_data = False
        self.start_time = time.time()
        
        # Crear carpeta de salida si no existe
        os.makedirs(self.output_folder, exist_ok=True)
        
        # Limpiar datos anteriores
        self.base_h_tool_accumulated = TransformArray()
        self.camera_h_charuco_accumulated = TransformArray()
        
        self.base_h_tool_accumulated.header = Header()
        self.base_h_tool_accumulated.header.stamp = self._node.get_clock().now().to_msg()
        self.base_h_tool_accumulated.header.frame_id = 'base_link'
        
        self.camera_h_charuco_accumulated.header = Header()
        self.camera_h_charuco_accumulated.header.stamp = self._node.get_clock().now().to_msg()
        self.camera_h_charuco_accumulated.header.frame_id = 'camera_color_optical_frame'
        
        # Verificar si ya hay detecciones guardadas
        detections_file = '/home/drims/drims_ws/calibrations/charuco_detections.yaml'
        if os.path.exists(detections_file):
            Logger.loginfo("📂 Cargando detecciones desde archivo existente...")
            if self._load_from_file():
                Logger.loginfo("✅ Detecciones cargadas desde archivo")
                self.received_data = True
                return
        
        # Si no hay archivo, lanzar nodo
        self._launch_charuco_node()
        
        # Crear suscripciones usando el nodo de FlexBE
        self.world_effector_sub = self._node.create_subscription(
            TransformArray,
            '/world_effector_poses',
            self.world_effector_callback,
            10
        )
        
        self.camera_object_sub = self._node.create_subscription(
            TransformArray,
            '/camera_object_poses',
            self.camera_object_callback,
            10
        )
        
        Logger.loginfo("⏳ Esperando detecciones del nodo charuco_hand_eye...")
    
    def _launch_charuco_node(self):
        """Lanza el nodo charuco_hand_eye"""
        try:
            # Verificar si ya está corriendo
            if self._is_charuco_running():
                Logger.loginfo("✅ Nodo charuco_hand_eye ya está corriendo")
                return
            
            Logger.loginfo("🚀 Lanzando nodo charuco_hand_eye...")
            
            # Convertir booleano a string 'true'/'false' para ROS2
            eye_in_hand_str = 'true' if self.eye_in_hand else 'false'
            
            cmd = [
                'ros2', 'run', 'charuco_calibrator', 'charuco_hand_eye',
                '--ros-args',
                '-p', f'pictures_folder:={self.pictures_folder}',
                '-p', f'robot_poses_folder:={self.robot_poses_folder}',
                '-p', f'output_folder:={self.output_folder}',
                '-p', f'eye_in_hand:={eye_in_hand_str}',
                '-p', 'camera_intrinsics_file:=/home/drims/drims_ws/calibrations/camera_intrinsics.yaml',
                '-p', 'publish_rate:=0.5',
                '-p', 'save_results:=True'
            ]
            
            Logger.loginfo(f"📋 Comando: {' '.join(cmd)}")
            
            self.charuco_process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                start_new_session=True,
                text=True
            )
            
            # Esperar a que inicie
            Logger.loginfo("⏳ Esperando a que el nodo Charuco inicie...")
            time.sleep(5)
            
            # Verificar si el proceso sigue vivo
            if self.charuco_process.poll() is not None:
                # El proceso terminó, leer error
                stdout, stderr = self.charuco_process.communicate()
                Logger.logerr(f"❌ El nodo Charuco terminó prematuramente")
                if stderr:
                    Logger.logerr(f"Error: {stderr}")
            
        except Exception as e:
            Logger.logerr(f"❌ Error lanzando charuco_hand_eye: {e}")
    
    def _is_charuco_running(self):
        """Verifica si el nodo está corriendo"""
        try:
            result = subprocess.run(
                ['ros2', 'node', 'list'],
                capture_output=True,
                text=True,
                timeout=2
            )
            node_list = result.stdout
            return any(name in node_list for name in ['hand_eye_calibrator', 'charuco_hand_eye'])
        except:
            return False
    
    def _load_from_file(self):
        """Carga detecciones desde archivo YAML"""
        try:
            detections_file = '/home/drims/drims_ws/calibrations/charuco_detections.yaml'
            
            if not os.path.exists(detections_file):
                return False
            
            with open(detections_file, 'r') as f:
                data = yaml.safe_load(f)
            
            if not data or 'pairs' not in data:
                return False
            
            # Construir mensajes TransformArray desde el archivo
            world_effector_msg = TransformArray()
            camera_object_msg = TransformArray()
            
            world_effector_msg.header = Header()
            world_effector_msg.header.stamp = self._node.get_clock().now().to_msg()
            world_effector_msg.header.frame_id = 'base_link'
            
            camera_object_msg.header = Header()
            camera_object_msg.header.stamp = self._node.get_clock().now().to_msg()
            camera_object_msg.header.frame_id = 'camera_color_optical_frame'
            
            for pair in data['pairs']:
                # Robot pose
                trans_robot = Transform()
                pos = pair['robot_position']
                quat = pair['robot_orientation']
                
                trans_robot.translation.x = float(pos[0])
                trans_robot.translation.y = float(pos[1])
                trans_robot.translation.z = float(pos[2])
                trans_robot.rotation.x = float(quat[0])
                trans_robot.rotation.y = float(quat[1])
                trans_robot.rotation.z = float(quat[2])
                trans_robot.rotation.w = float(quat[3])
                
                # Charuco pose
                trans_charuco = Transform()
                t = pair['charuco_translation']
                R = np.array(pair['charuco_rotation_matrix'])
                
                # Matriz de rotación a cuaternión
                T = np.eye(4)
                T[:3, :3] = R
                quat_charuco = tf_transformations.quaternion_from_matrix(T)
                
                trans_charuco.translation.x = float(t[0])
                trans_charuco.translation.y = float(t[1])
                trans_charuco.translation.z = float(t[2])
                trans_charuco.rotation.x = float(quat_charuco[0])
                trans_charuco.rotation.y = float(quat_charuco[1])
                trans_charuco.rotation.z = float(quat_charuco[2])
                trans_charuco.rotation.w = float(quat_charuco[3])
                
                world_effector_msg.transforms.append(trans_robot)
                camera_object_msg.transforms.append(trans_charuco)
            
            self.base_h_tool_accumulated = world_effector_msg
            self.camera_h_charuco_accumulated = camera_object_msg
            
            Logger.loginfo(f"✅ Cargados {len(world_effector_msg.transforms)} pares desde archivo")
            return True
            
        except Exception as e:
            Logger.logwarn(f"⚠️ Error cargando archivo: {e}")
            return False
    
    def world_effector_callback(self, msg):
        """Callback para poses del robot"""
        self.base_h_tool_accumulated = msg
        self._check_complete()
    
    def camera_object_callback(self, msg):
        """Callback para poses del charuco"""
        self.camera_h_charuco_accumulated = msg
        self._check_complete()
    
    def _check_complete(self):
        """Verifica si ya tenemos todos los datos"""
        if (hasattr(self.base_h_tool_accumulated, 'transforms') and 
            hasattr(self.camera_h_charuco_accumulated, 'transforms') and
            len(self.base_h_tool_accumulated.transforms) > 0 and 
            len(self.camera_h_charuco_accumulated.transforms) > 0 and
            len(self.base_h_tool_accumulated.transforms) == len(self.camera_h_charuco_accumulated.transforms)):
            self.received_data = True
            Logger.loginfo(f"✅ Recibidos {len(self.base_h_tool_accumulated.transforms)} pares de calibración")
    
    def execute(self, userdata):
        """Ejecución: esperar datos o timeout"""
        if self.received_data:
            userdata.base_h_tool_accumulated = self.base_h_tool_accumulated
            userdata.camera_h_charuco_accumulated = self.camera_h_charuco_accumulated
            return 'completed'
        
        # Verificar timeout
        elapsed = time.time() - self.start_time
        if elapsed > self.max_wait_time:
            Logger.logerr(f"❌ Timeout esperando detecciones ({self.max_wait_time}s)")
            return 'failed'
        
        return None  # Seguir esperando
    
    def on_stop(self):
        """Limpiar al terminar"""
        # Cancelar suscripciones
        if self.world_effector_sub:
            self._node.destroy_subscription(self.world_effector_sub)
        if self.camera_object_sub:
            self._node.destroy_subscription(self.camera_object_sub)
        
        # Detener proceso Charuco si lo iniciamos
        if self.charuco_process:
            Logger.loginfo("🛑 Deteniendo nodo Charuco...")
            try:
                # Enviar SIGINT
                os.killpg(os.getpgid(self.charuco_process.pid), signal.SIGINT)
                self.charuco_process.wait(timeout=5)
            except:
                try:
                    # Forzar terminación
                    self.charuco_process.terminate()
                    self.charuco_process.wait(timeout=3)
                except:
                    try:
                        self.charuco_process.kill()
                    except:
                        pass
            self.charuco_process = None
            Logger.loginfo("✅ Nodo Charuco detenido")
