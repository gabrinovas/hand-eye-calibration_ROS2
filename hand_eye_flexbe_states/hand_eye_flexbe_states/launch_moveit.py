#!/usr/bin/env python3
"""
Estado FlexBE para lanzar MoveIt automáticamente.
"""

from flexbe_core import EventState, Logger
import subprocess
import time
import os
import psutil
from ament_index_python.packages import get_package_share_directory

class LaunchMoveItState(EventState):
    """
    Lanza MoveIt con la interfaz gráfica.
    
    -- moveit_launch_file    string   Archivo launch de MoveIt
    -- robot_name            string   Nombre del robot
    -- moveit_config_package string   Paquete de configuración de MoveIt
    
    <= done                     MoveIt lanzado correctamente
    <= failed                   Error al lanzar MoveIt
    """
    
    def __init__(self, moveit_launch_file='move_group.launch.py', 
                 robot_name='panda',
                 moveit_config_package='panda_moveit_config'):
        super().__init__(outcomes=['done', 'failed'])
        
        self.moveit_launch_file = moveit_launch_file
        self.robot_name = robot_name
        self.moveit_config_package = moveit_config_package
        self.process = None
        
    def on_start(self):
        """Lanzar MoveIt al iniciar el estado"""
        try:
            # Verificar si MoveIt ya está corriendo
            if self._is_moveit_running():
                Logger.loginfo("✅ MoveIt ya está corriendo")
                return
            
            Logger.loginfo("🚀 Lanzando MoveIt...")
            
            # Construir comando
            cmd = [
                'ros2', 'launch',
                self.moveit_config_package,
                self.moveit_launch_file
            ]
            
            # Opciones adicionales según robot
            if self.robot_name == 'panda':
                cmd.extend(['robot_ip:=192.168.1.100'])  # Ajustar según configuración
            elif self.robot_name == 'ur':
                cmd.extend(['ur_type:=ur5e'])
            
            # Lanzar en proceso separado
            self.process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                start_new_session=True
            )
            
            # Esperar a que inicie
            Logger.loginfo("⏳ Esperando a que MoveIt inicie...")
            time.sleep(5)
            
            # Verificar que inició
            if self._is_moveit_running():
                Logger.loginfo("✅ MoveIt lanzado correctamente")
                self._launch_rviz()
            else:
                stderr = self.process.stderr.read().decode()
                Logger.logerr(f"❌ Error lanzando MoveIt: {stderr}")
                
        except Exception as e:
            Logger.logerr(f"❌ Error: {str(e)}")
    
    def _is_moveit_running(self):
        """Verifica si MoveIt está corriendo"""
        try:
            for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
                try:
                    cmdline = proc.info['cmdline']
                    if cmdline and any('move_group' in ' '.join(cmdline) for cmdline in [cmdline]):
                        return True
                except:
                    pass
        except:
            pass
        return False
    
    def _launch_rviz(self):
        """Lanza RViz si no está abierto"""
        try:
            for proc in psutil.process_iter(['pid', 'name']):
                try:
                    if 'rviz2' in proc.info['name']:
                        Logger.loginfo("✅ RViz ya está abierto")
                        return
                except:
                    pass
            
            Logger.loginfo("🖥️ Abriendo RViz...")
            rviz_cmd = [
                'ros2', 'run', 'rviz2', 'rviz2',
                '-d', os.path.join(
                    get_package_share_directory(self.moveit_config_package),
                    'config',
                    'moveit.rviz'
                )
            ]
            
            subprocess.Popen(
                rviz_cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                start_new_session=True
            )
            
            time.sleep(2)
            Logger.loginfo("✅ RViz lanzado")
            
        except Exception as e:
            Logger.logwarn(f"⚠️ No se pudo lanzar RViz: {e}")
    
    def execute(self, userdata):
        return 'done'
    
    def on_stop(self):
        Logger.loginfo("ℹ️ MoveIt sigue corriendo para futuros usos")
