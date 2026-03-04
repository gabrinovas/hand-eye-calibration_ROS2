#!/usr/bin/env python3
"""
Estado FlexBE para lanzar MoveIt automáticamente para UR5e.
"""

from flexbe_core import EventState, Logger
import subprocess
import time
import os
import psutil
import signal
from ament_index_python.packages import get_package_share_directory

class LaunchMoveItState(EventState):
    """
    Lanza MoveIt con la interfaz gráfica para UR5e.
    
    -- moveit_launch_file    string   Archivo launch de MoveIt
    -- robot_name            string   Nombre del robot (ur5e)
    -- moveit_config_package string   Paquete de configuración (ur_moveit_config)
    -- robot_ip              string   IP del robot UR
    -- use_fake_hardware     bool     Usar simulación o robot real
    
    <= done                     MoveIt lanzado correctamente
    <= failed                   Error al lanzar MoveIt
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
        
    def on_start(self):
        """Lanzar MoveIt al iniciar el estado"""
        try:
            # Verificar si MoveIt ya está corriendo
            if self._is_moveit_running():
                Logger.loginfo("✅ MoveIt ya está corriendo")
                return
            
            Logger.loginfo("="*60)
            Logger.loginfo("🚀 Lanzando MoveIt para UR5e...")
            Logger.loginfo("="*60)
            Logger.loginfo(f"📦 Paquete: {self.moveit_config_package}")
            Logger.loginfo(f"📄 Launch: {self.moveit_launch_file}")
            Logger.loginfo(f"🤖 Robot: {self.robot_name}")
            
            if self.use_fake_hardware:
                Logger.loginfo("🖥️ Modo: SIMULACIÓN (fake hardware)")
            else:
                Logger.loginfo(f"🔌 Modo: ROBOT REAL (IP: {self.robot_ip})")
            
            # Construir comando para UR
            cmd = [
                'ros2', 'launch',
                self.moveit_config_package,
                self.moveit_launch_file
            ]
            
            # Añadir argumentos específicos para UR
            if self.use_fake_hardware:
                cmd.extend(['use_fake_hardware:=true'])
                cmd.extend(['robot_ip:=xxx.xxx.xxx.xxx'])  # Placeholder para fake
            else:
                cmd.extend(['use_fake_hardware:=false'])
                cmd.extend([f'robot_ip:={self.robot_ip}'])
            
            # Añadir tipo de UR
            cmd.extend([f'ur_type:={self.robot_name}'])
            
            # Opciones adicionales útiles
            cmd.extend(['launch_rviz:=true'])
            cmd.extend(['headless:=false'])
            
            Logger.loginfo(f"📋 Comando: {' '.join(cmd)}")
            
            # Lanzar en proceso separado
            self.process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                start_new_session=True,
                text=True
            )
            
            # Esperar a que inicie
            Logger.loginfo("⏳ Esperando a que MoveIt inicie (30 segundos)...")
            
            # Monitorear inicio
            for i in range(30):
                time.sleep(1)
                if self._is_moveit_running():
                    Logger.loginfo(f"✅ MoveIt lanzado correctamente después de {i+1}s")
                    
                    # Verificar RViz
                    time.sleep(2)
                    if self._is_rviz_running():
                        Logger.loginfo("✅ RViz está corriendo")
                    else:
                        Logger.logwarn("⚠️ RViz no detectado, puede estar iniciando...")
                    
                    return
                
                # Verificar si el proceso falló
                if self.process.poll() is not None:
                    stderr = self.process.stderr.read()
                    Logger.logerr(f"❌ Error lanzando MoveIt: {stderr}")
                    return 'failed'
            
            # Si llegamos aquí, timeout
            Logger.logerr("❌ Timeout esperando a MoveIt")
            return 'failed'
            
        except Exception as e:
            Logger.logerr(f"❌ Error: {str(e)}")
            return 'failed'
    
    def _is_moveit_running(self):
        """Verifica si MoveIt está corriendo"""
        try:
            # Buscar nodos de MoveIt
            result = subprocess.run(
                ['ros2', 'node', 'list'],
                capture_output=True,
                text=True,
                timeout=5
            )
            
            moveit_nodes = ['move_group', 'robot_state_publisher']
            for node in moveit_nodes:
                if node in result.stdout:
                    return True
                    
        except Exception as e:
            Logger.logwarn(f"⚠️ Error verificando MoveIt: {e}")
        
        return False
    
    def _is_rviz_running(self):
        """Verifica si RViz está corriendo"""
        try:
            for proc in psutil.process_iter(['pid', 'name']):
                try:
                    if 'rviz2' in proc.info['name'].lower():
                        return True
                except:
                    pass
        except:
            pass
        return False
    
    def execute(self, userdata):
        """Ejecutar - siempre retorna done si llegamos aquí"""
        return 'done'
    
    def on_stop(self):
        """Al detener el comportamiento"""
        Logger.loginfo("ℹ️ MoveIt sigue corriendo para futuros usos")
        Logger.loginfo("   Para detenerlo manualmente: pkill -f move_group")
