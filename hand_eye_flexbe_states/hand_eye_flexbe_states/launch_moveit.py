#!/usr/bin/env python3
"""
Estado FlexBE para lanzar MoveIt automáticamente para UR5e.
"""

from flexbe_core import EventState, Logger
import subprocess
import time
import os
import psutil
from ament_index_python.packages import get_package_share_directory

class LaunchMoveItState(EventState):
    """
    Lanza MoveIt con la interfaz gráfica para UR5e.
    
    -- moveit_launch_file    string   Archivo launch de MoveIt
    -- robot_name            string   Nombre del robot (ur5e)
    -- moveit_config_package string   Paquete de configuración
    -- robot_ip              string   IP del robot UR
    -- use_fake_hardware     bool     Usar simulación o robot real
    
    <= done                     MoveIt lanzado (aunque no esté 100% listo)
    <= failed                   Error al lanzar
    """
    
    def __init__(self, moveit_launch_file='ur_moveit.launch.py', 
                 robot_name='ur5e',
                 moveit_config_package='ur_moveit_config',
                 robot_ip='192.168.1.101',
                 use_fake_hardware=True):
        super().__init__(outcomes=['done', 'failed'])
        
        self.moveit_launch_file = moveit_launch_file
        self.robot_name = robot_name
        self.moveit_config_package = moveit_config_package
        self.robot_ip = robot_ip
        self.use_fake_hardware = use_fake_hardware
        self.process = None
        self.launch_attempted = False
        
    def on_start(self):
        """Lanzar MoveIt al iniciar el estado"""
        if self.launch_attempted:
            return
            
        self.launch_attempted = True
        
        try:
            # Verificación rápida: si ya hay nodos de MoveIt, no lanzamos
            if self._is_moveit_basically_running():
                Logger.loginfo("✅ MoveIt ya está corriendo")
                return
            
            Logger.loginfo("="*60)
            Logger.loginfo("🚀 Lanzando MoveIt para UR5e...")
            Logger.loginfo("="*60)
            
            # Comando simplificado
            cmd = [
                'ros2', 'launch',
                self.moveit_config_package,
                self.moveit_launch_file,
                f'ur_type:={self.robot_name}',
                f'robot_ip:={self.robot_ip}',
                f'use_fake_hardware:={str(self.use_fake_hardware).lower()}',
                'launch_rviz:=true'
            ]
            
            Logger.loginfo(f"📋 Comando: {' '.join(cmd)}")
            
            # Variables de entorno
            env = os.environ.copy()
            env['DISPLAY'] = ':0'
            
            # Lanzar en proceso separado
            self.process = subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                start_new_session=True,
                env=env
            )
            
            # Esperar SOLO 3 segundos para que inicie lo básico
            Logger.loginfo("⏳ Esperando 3 segundos para que MoveIt inicie...")
            time.sleep(3)
            
            Logger.loginfo("✅ MoveIt lanzado, continuando con la captura...")
            
        except Exception as e:
            Logger.logerr(f"❌ Error lanzando MoveIt: {str(e)}")
    
    def _is_moveit_basically_running(self):
        """Verificación rápida: solo mira si existe el nodo move_group"""
        try:
            result = subprocess.run(
                ['ros2', 'node', 'list'],
                capture_output=True,
                text=True,
                timeout=2
            )
            return 'move_group' in result.stdout
        except:
            return False
    
    def execute(self, userdata):
        """Siempre retorna done para continuar con la captura"""
        return 'done'
    
    def on_stop(self):
        """No hacemos nada, MoveIt sigue corriendo"""
        pass
