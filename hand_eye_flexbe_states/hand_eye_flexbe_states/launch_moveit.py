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
    
    -- moveit_launch_file    string   Archivo launch de MoveIt (ur_moveit.launch.py)
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
                # Verificar RViz específicamente
                if not self._is_rviz_running():
                    Logger.loginfo("🖥️ RViz no está abierto, lanzándolo...")
                    self._launch_rviz()
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
                self.moveit_launch_file,
                f'ur_type:={self.robot_name}',
                f'robot_ip:={self.robot_ip}',
                f'use_fake_hardware:={str(self.use_fake_hardware).lower()}',
                'launch_rviz:=true'
            ]
            
            Logger.loginfo(f"📋 Comando: {' '.join(cmd)}")
            
            # Variables de entorno para forzar visualización
            env = os.environ.copy()
            env['DISPLAY'] = ':0'  # Asegurar que usa el display correcto
            env['XDG_RUNTIME_DIR'] = f'/run/user/{os.getuid()}'  # Para problemas de permisos
            
            # Lanzar en proceso separado
            self.process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                start_new_session=True,
                text=True,
                env=env
            )
            
            # Esperar a que inicie
            Logger.loginfo("⏳ Esperando a que MoveIt inicie (30 segundos)...")
            
            # Monitorear inicio
            moveit_started = False
            for i in range(30):
                time.sleep(1)
                
                # Verificar nodos de MoveIt
                if self._is_moveit_running():
                    if not moveit_started:
                        Logger.loginfo(f"✅ MoveIt lanzado correctamente después de {i+1}s")
                        moveit_started = True
                    
                    # Verificar RViz
                    if self._is_rviz_running():
                        Logger.loginfo("✅ RViz está corriendo")
                        return
                    else:
                        # Si MoveIt está pero RViz no, esperar un poco más
                        if i > 10:  # Esperar 10 segundos antes de preocuparse
                            Logger.logwarn("⚠️ RViz no detectado, puede estar iniciando...")
                
                # Verificar si el proceso falló
                if self.process.poll() is not None:
                    stderr = self.process.stderr.read()
                    stdout = self.process.stdout.read()
                    Logger.logerr(f"❌ Error lanzando MoveIt:")
                    Logger.logerr(f"STDERR: {stderr}")
                    Logger.logerr(f"STDOUT: {stdout}")
                    return 'failed'
            
            # Si llegamos aquí, timeout pero MoveIt puede estar corriendo
            if moveit_started:
                Logger.logwarn("⚠️ MoveIt iniciado pero timeout esperando confirmación")
                # Intentar lanzar RViz explícitamente por si acaso
                if not self._is_rviz_running():
                    self._launch_rviz()
                return
            else:
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
            
            # Nodos clave de MoveIt para UR
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
            # Buscar por proceso
            for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
                try:
                    if 'rviz2' in proc.info['name'].lower():
                        return True
                    # También buscar en cmdline por si el nombre es diferente
                    if proc.info['cmdline'] and any('rviz2' in arg for arg in proc.info['cmdline']):
                        return True
                except:
                    pass
            
            # También buscar ventanas (Linux)
            try:
                result = subprocess.run(
                    ['wmctrl', '-l'],
                    capture_output=True,
                    text=True,
                    timeout=2
                )
                if 'RViz' in result.stdout or 'rviz' in result.stdout.lower():
                    return True
            except:
                pass
                
        except Exception as e:
            Logger.logwarn(f"⚠️ Error verificando RViz: {e}")
        
        return False
    
    def _launch_rviz(self):
        """Lanza RViz explícitamente si no está abierto"""
        try:
            # Buscar archivo de configuración de RViz
            rviz_config = None
            possible_paths = [
                os.path.join(get_package_share_directory(self.moveit_config_package), 'config', 'moveit.rviz'),
                os.path.join(get_package_share_directory(self.moveit_config_package), 'config', 'ur5e.rviz'),
                os.path.join(get_package_share_directory(self.moveit_config_package), 'rviz', 'view_robot.rviz'),
                os.path.join(get_package_share_directory(self.moveit_config_package), 'launch', 'moveit.rviz'),
            ]
            
            for path in possible_paths:
                if os.path.exists(path):
                    rviz_config = path
                    Logger.loginfo(f"✅ Encontrada configuración RViz: {path}")
                    break
            
            cmd = ['ros2', 'run', 'rviz2', 'rviz2']
            if rviz_config:
                cmd.extend(['-d', rviz_config])
            else:
                Logger.logwarn("⚠️ No se encontró archivo de configuración de RViz, usando defaults")
            
            # Variables de entorno
            env = os.environ.copy()
            env['DISPLAY'] = ':0'
            env['XDG_RUNTIME_DIR'] = f'/run/user/{os.getuid()}'
            
            Logger.loginfo(f"🖥️ Lanzando RViz: {' '.join(cmd)}")
            
            subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                start_new_session=True,
                env=env
            )
            
            time.sleep(3)
            
            if self._is_rviz_running():
                Logger.loginfo("✅ RViz lanzado correctamente")
            else:
                Logger.logwarn("⚠️ RViz puede estar iniciando...")
            
        except Exception as e:
            Logger.logwarn(f"⚠️ Error lanzando RViz: {e}")
    
    def execute(self, userdata):
        """Ejecutar - siempre retorna done si llegamos aquí"""
        return 'done'
    
    def on_stop(self):
        """Al detener el comportamiento"""
        Logger.loginfo("ℹ️ MoveIt sigue corriendo para futuros usos")
        Logger.loginfo("   Para detenerlo manualmente: pkill -f move_group")
