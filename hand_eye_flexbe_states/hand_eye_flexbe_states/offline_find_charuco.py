#!/usr/bin/env python3
from flexbe_core import EventState, Logger
import os
import yaml
import glob
from geometry_msgs.msg import Transform
from visp_hand2eye_calibration.msg import TransformArray
from std_msgs.msg import Header

class OfflineFindCharucoState(EventState):
    """
    Procesa offline las poses de Charuco desde archivos.
    
    -- pictures_folder   string   Carpeta con imágenes
    -- robot_poses_folder string   Carpeta con poses del robot
    -- eye_in_hand       bool     Modo eye-in-hand o eye-to-hand
    
    ># current_index      int      Índice actual
    #> base_h_tool        TransformArray  Pose actual
    #> camera_h_charuco    TransformArray  Pose actual
    #> base_h_tool_accumulated  TransformArray  TODAS las poses
    #> camera_h_charuco_accumulated TransformArray TODAS las poses
    
    <= done                 Siguiente pose
    <= completed           Todas procesadas
    <= failed              Error
    """
    
    def __init__(self, pictures_folder, robot_poses_folder, eye_in_hand=False):
        super().__init__(
            outcomes=['done', 'completed', 'failed'],
            input_keys=['current_index'],
            output_keys=['base_h_tool', 'camera_h_charuco', 
                        'base_h_tool_accumulated', 'camera_h_charuco_accumulated']
        )
        
        self.pictures_folder = pictures_folder
        self.robot_poses_folder = robot_poses_folder
        self.eye_in_hand = eye_in_hand
        
        self.pairs = []
        self.base_all = TransformArray()
        self.camera_all = TransformArray()
        self.loaded = False
        
    def on_enter(self, userdata):
        if not self.loaded:
            if not self._load_pairs():
                return 'failed'
            self.loaded = True
            
            self.base_all.header = Header()
            self.base_all.header.frame_id = 'base_link'
            self.camera_all.header = Header()
            self.camera_all.header.frame_id = 'calib_camera'
        
        if userdata.current_index >= len(self.pairs):
            userdata.base_h_tool_accumulated = self.base_all
            userdata.camera_h_charuco_accumulated = self.camera_all
            Logger.loginfo(f"✅ Procesadas {len(self.base_all.transforms)} poses")
            return 'completed'
        
        if not self._load_pair(userdata.current_index, userdata):
            return 'failed'
        
        userdata.current_index += 1
        return 'done'
    
    def _load_pairs(self):
        """Carga los pares generados"""
        try:
            pairs_folder = os.path.join(os.path.dirname(self.robot_poses_folder), 'pairs')
            if os.path.exists(pairs_folder):
                pair_files = sorted(glob.glob(os.path.join(pairs_folder, 'pair_*.yaml')))
                for pf in pair_files:
                    with open(pf, 'r') as f:
                        self.pairs.append(yaml.safe_load(f))
            
            Logger.loginfo(f"📚 Cargados {len(self.pairs)} pares")
            return True
        except Exception as e:
            Logger.logerr(f"❌ Error cargando pares: {e}")
            return False
    
    def _load_pair(self, index, userdata):
        """Carga un par específico"""
        try:
            pair = self.pairs[index]
            
            # Pose robot
            trans_robot = Transform()
            robot_pose = pair['robot_pose']
            trans_robot.translation.x = robot_pose['position'][0]
            trans_robot.translation.y = robot_pose['position'][1]
            trans_robot.translation.z = robot_pose['position'][2]
            trans_robot.rotation.x = robot_pose['orientation'][0]
            trans_robot.rotation.y = robot_pose['orientation'][1]
            trans_robot.rotation.z = robot_pose['orientation'][2]
            trans_robot.rotation.w = robot_pose['orientation'][3]
            
            # Pose Charuco
            trans_charuco = Transform()
            charuco_pose = pair['charuco_pose']
            trans_charuco.translation.x = charuco_pose['translation'][0]
            trans_charuco.translation.y = charuco_pose['translation'][1]
            trans_charuco.translation.z = charuco_pose['translation'][2]
            trans_charuco.rotation.x = charuco_pose['quaternion'][0]
            trans_charuco.rotation.y = charuco_pose['quaternion'][1]
            trans_charuco.rotation.z = charuco_pose['quaternion'][2]
            trans_charuco.rotation.w = charuco_pose['quaternion'][3]
            
            # Acumular
            self.base_all.transforms.append(trans_robot)
            self.camera_all.transforms.append(trans_charuco)
            
            # Pasar actual
            base_current = TransformArray()
            base_current.header = self.base_all.header
            base_current.transforms = [trans_robot]
            
            camera_current = TransformArray()
            camera_current.header = self.camera_all.header
            camera_current.transforms = [trans_charuco]
            
            userdata.base_h_tool = base_current
            userdata.camera_h_charuco = camera_current
            
            return True
            
        except Exception as e:
            Logger.logerr(f"❌ Error en par {index}: {e}")
            return False
