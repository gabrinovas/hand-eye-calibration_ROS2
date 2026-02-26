#!/usr/bin/env python3
"""
Estado FlexBE para procesar offline las poses de Charuco desde archivos.
Acumula todas las poses y las pasa al estado de calibración.
"""

from flexbe_core import EventState, Logger
import os
import yaml
import glob
from geometry_msgs.msg import Transform
from visp_hand2eye_calibration.msg import TransformArray
from std_msgs.msg import Header

class OfflineFindCharucoState(EventState):
    """
    Procesa offline las poses de Charuco desde archivos generados.
    
    -- pictures_folder   string   Carpeta con imágenes
    -- robot_poses_folder string   Carpeta con poses del robot
    -- output_folder     string   Carpeta de salida
    -- eye_in_hand       bool     Modo eye-in-hand o eye-to-hand
    
    ># current_index      int      Índice actual
    #> base_h_tool        TransformArray  Pose actual (para debug)
    #> camera_h_charuco    TransformArray  Pose actual (para debug)
    #> base_h_tool_accumulated  TransformArray  TODAS las poses
    #> camera_h_charuco_accumulated TransformArray TODAS las poses
    
    <= done                 Siguiente pose
    <= completed           Todas procesadas
    <= failed              Error
    """
    
    def __init__(self, pictures_folder, robot_poses_folder, output_folder=None, eye_in_hand=False):
        super().__init__(
            outcomes=['done', 'completed', 'failed'],
            input_keys=['current_index'],
            output_keys=['base_h_tool', 'camera_h_charuco', 
                        'base_h_tool_accumulated', 'camera_h_charuco_accumulated']
        )
        
        self.pictures_folder = pictures_folder
        self.robot_poses_folder = robot_poses_folder
        self.output_folder = output_folder or '/home/drims/drims_ws/calibrations/extrinsic_calib_charuco_poses'
        self.eye_in_hand = eye_in_hand
        
        self.pairs = []
        self.base_all = TransformArray()
        self.camera_all = TransformArray()
        self.loaded = False
        
    def on_enter(self, userdata):
        """Al entrar, cargar pares y procesar"""
        if not self.loaded:
            if not self._load_pairs():
                return 'failed'
            self.loaded = True
            
            # Inicializar acumuladores
            self.base_all.header = Header()
            self.base_all.header.stamp = self._node.get_clock().now().to_msg()
            self.base_all.header.frame_id = 'base_link'
            
            self.camera_all.header = Header()
            self.camera_all.header.stamp = self._node.get_clock().now().to_msg()
            self.camera_all.header.frame_id = 'calib_camera'
            
            Logger.loginfo(f"📚 Cargados {len(self.pairs)} pares de calibración")
        
        # Verificar índice
        if 'current_index' not in userdata or userdata.current_index is None:
            userdata.current_index = 0
        
        if userdata.current_index >= len(self.pairs):
            # Terminamos: pasar acumuladores
            userdata.base_h_tool_accumulated = self.base_all
            userdata.camera_h_charuco_accumulated = self.camera_all
            Logger.loginfo(f"✅ Procesamiento completado: {len(self.base_all.transforms)} poses")
            return 'completed'
        
        # Cargar par actual
        if not self._load_pair(userdata.current_index, userdata):
            return 'failed'
        
        userdata.current_index += 1
        return 'done'
    
    def _load_pairs(self):
        """Carga los pares generados por generate_calibration_pairs.py"""
        try:
            # Buscar en pairs/
            pairs_folder = os.path.join(self.output_folder, 'pairs')
            if os.path.exists(pairs_folder):
                pair_files = sorted(glob.glob(os.path.join(pairs_folder, 'pair_*.yaml')))
                for pf in pair_files:
                    with open(pf, 'r') as f:
                        self.pairs.append(yaml.safe_load(f))
            
            # Si no hay pares, intentar cargar desde robot_poses_folder
            if not self.pairs:
                Logger.logwarn("⚠️ No se encontraron pares, buscando poses individuales...")
                pose_files = sorted(glob.glob(os.path.join(self.robot_poses_folder, 'pose_*.yaml')))
                detection_files = sorted(glob.glob(os.path.join(self.output_folder, 'detections', 'detection_*.yaml')))
                
                # Emparejar por índice
                for i, (pf, df) in enumerate(zip(pose_files, detection_files)):
                    with open(pf, 'r') as f:
                        pose_data = yaml.safe_load(f)
                    with open(df, 'r') as f:
                        det_data = yaml.safe_load(f)
                    
                    pair = {
                        'index': i+1,
                        'robot_pose': pose_data,
                        'charuco_pose': det_data
                    }
                    self.pairs.append(pair)
            
            return len(self.pairs) > 0
            
        except Exception as e:
            Logger.logerr(f"❌ Error cargando pares: {e}")
            return False
    
    def _load_pair(self, index, userdata):
        """Carga un par específico y lo acumula"""
        try:
            pair = self.pairs[index]
            
            # ===== POSE DEL ROBOT (base → tool) =====
            trans_robot = Transform()
            
            if 'robot_pose' in pair:
                robot_pose = pair['robot_pose']
                # Formato del robot_pose
                if 'position' in robot_pose:
                    trans_robot.translation.x = robot_pose['position'][0]
                    trans_robot.translation.y = robot_pose['position'][1]
                    trans_robot.translation.z = robot_pose['position'][2]
                else:
                    trans_robot.translation.x = robot_pose.get('x', 0)
                    trans_robot.translation.y = robot_pose.get('y', 0)
                    trans_robot.translation.z = robot_pose.get('z', 0)
                
                if 'orientation' in robot_pose:
                    trans_robot.rotation.x = robot_pose['orientation'][0]
                    trans_robot.rotation.y = robot_pose['orientation'][1]
                    trans_robot.rotation.z = robot_pose['orientation'][2]
                    trans_robot.rotation.w = robot_pose['orientation'][3]
                else:
                    trans_robot.rotation.x = robot_pose.get('qx', 0)
                    trans_robot.rotation.y = robot_pose.get('qy', 0)
                    trans_robot.rotation.z = robot_pose.get('qz', 0)
                    trans_robot.rotation.w = robot_pose.get('qw', 1)
            
            # ===== POSE DEL CHARUCO (cámara → charuco) =====
            trans_charuco = Transform()
            
            if 'charuco_pose' in pair:
                charuco_pose = pair['charuco_pose']
                if 'translation' in charuco_pose:
                    trans_charuco.translation.x = charuco_pose['translation'][0]
                    trans_charuco.translation.y = charuco_pose['translation'][1]
                    trans_charuco.translation.z = charuco_pose['translation'][2]
                else:
                    trans_charuco.translation.x = charuco_pose.get('x', 0)
                    trans_charuco.translation.y = charuco_pose.get('y', 0)
                    trans_charuco.translation.z = charuco_pose.get('z', 0)
                
                if 'quaternion' in charuco_pose:
                    trans_charuco.rotation.x = charuco_pose['quaternion'][0]
                    trans_charuco.rotation.y = charuco_pose['quaternion'][1]
                    trans_charuco.rotation.z = charuco_pose['quaternion'][2]
                    trans_charuco.rotation.w = charuco_pose['quaternion'][3]
                else:
                    trans_charuco.rotation.x = charuco_pose.get('qx', 0)
                    trans_charuco.rotation.y = charuco_pose.get('qy', 0)
                    trans_charuco.rotation.z = charuco_pose.get('qz', 0)
                    trans_charuco.rotation.w = charuco_pose.get('qw', 1)
            
            # ACUMULAR
            self.base_all.transforms.append(trans_robot)
            self.camera_all.transforms.append(trans_charuco)
            
            # Pasar pose actual para debug
            base_current = TransformArray()
            base_current.header = self.base_all.header
            base_current.transforms = [trans_robot]
            
            camera_current = TransformArray()
            camera_current.header = self.camera_all.header
            camera_current.transforms = [trans_charuco]
            
            userdata.base_h_tool = base_current
            userdata.camera_h_charuco = camera_current
            
            Logger.loginfo(f"✅ Par {index+1}/{len(self.pairs)} cargado")
            
            return True
            
        except Exception as e:
            Logger.logerr(f"❌ Error cargando par {index}: {e}")
            return False
