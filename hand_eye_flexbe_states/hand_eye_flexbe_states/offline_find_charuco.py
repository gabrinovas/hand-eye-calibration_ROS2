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
import rclpy
from rclpy.node import Node
from visp_hand2eye_calibration.msg import TransformArray
from std_msgs.msg import Header
import psutil

class OfflineFindCharucoState(EventState):
    """
    Inicia el nodo de detección de Charuco y espera a que publique.
    """
    
    def __init__(self, pictures_folder, robot_poses_folder, eye_in_hand=False):
        super().__init__(
            outcomes=['completed', 'failed'],
            output_keys=['base_h_tool_accumulated', 'camera_h_charuco_accumulated']
        )
        
        self.pictures_folder = pictures_folder
        self.robot_poses_folder = robot_poses_folder
        self.eye_in_hand = eye_in_hand
        
        self.charuco_process = None
        self.base_h_tool_accumulated = None
        self.camera_h_charuco_accumulated = None
        self.received_data = False
        self.max_wait_time = 60  # segundos máximo de espera
        self.start_time = None
        
        # Suscripciones
        self.world_effector_sub = None
        self.camera_object_sub = None
        
        Logger.loginfo("="*60)
        Logger.loginfo("🔍 ESTADO: OfflineFindCharuco")
        Logger.loginfo("="*60)
        Logger.loginfo(f"📁 Imágenes: {self.pictures_folder}")
        Logger.loginfo(f"📁 Poses robot: {self.robot_poses_folder}")
        Logger.loginfo(f"🎯 Modo: {'Eye-in-hand' if eye_in_hand else 'Eye-to-hand'}")
        
    def on_enter(self, userdata):
        """Iniciar nodo de detección y suscribirse"""
        self.received_data = False
        self.start_time = time.time()
        
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
        
        # Crear suscripciones
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
            
            cmd = [
                'ros2', 'run', 'charuco_calibrator', 'charuco_hand_eye_offline',
                '--ros-args',
                '-p', f'pictures_folder:={self.pictures_folder}',
                '-p', f'robot_poses_folder:={self.robot_poses_folder}',
                '-p', f'eye_in_hand:={str(self.eye_in_hand).lower()}',
                '-p', 'publish_rate:=0.5'  # Publica cada 2 segundos
            ]
            
            Logger.loginfo(f"📋 Comando: {' '.join(cmd)}")
            
            self.charuco_process = subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                start_new_session=True
            )
            
            # Esperar a que inicie
            time.sleep(3)
            
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
            return 'hand_eye_calibrator' in result.stdout
        except:
            return False
    
    def _load_from_file(self):
        """Carga detecciones desde archivo YAML"""
        try:
            detections_file = '/home/drims/drims_ws/calibrations/charuco_detections.yaml'
            poses_file = '/home/drims/drims_ws/calibrations/robot_poses.yaml'
            
            with open(detections_file, 'r') as f:
                detections = yaml.safe_load(f)
            
            # Aquí construirías los mensajes TransformArray desde el archivo
            # Por simplicidad, mejor esperar a que publique el nodo
            return False
            
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
        if (len(self.base_h_tool_accumulated.transforms) > 0 and 
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
        if time.time() - self.start_time > self.max_wait_time:
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
        
        # No matamos el nodo charuco, podría estar siendo usado
        self.charuco_process = None
