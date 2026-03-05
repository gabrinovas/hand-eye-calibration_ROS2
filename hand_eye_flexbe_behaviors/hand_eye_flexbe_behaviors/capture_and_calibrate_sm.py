#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, Logger
from hand_eye_flexbe_states.launch_moveit import LaunchMoveItState
from hand_eye_flexbe_states.take_pose_and_picture import TakePoseAndPictureState
from hand_eye_flexbe_states.offline_find_charuco import OfflineFindCharucoState
from hand_eye_flexbe_states.compute_calib import ComputeCalibState
# [MANUAL_IMPORT]
# [/MANUAL_IMPORT]


'''
Created on Thu Feb 26 2026
@author: Gabriel Novas
'''
class CaptureAndCalibrateSM(Behavior):
    """
    Comportamiento COMPLETO para calibración ojo-mano:
    
    FASE 1: Lanza MoveIt + RViz automáticamente
    FASE 2: Tú controlas el robot con la interfaz de MoveIt
            Presionas ESPACIO en cada pose para capturar
    FASE 3: Procesamiento offline con charuco_hand_eye
    FASE 4: Cálculo automático de calibración con VISP
    
    RESULTADOS: /home/drims/drims_ws/calibrations/hand_eye_calibration.yaml
    """
    
    def __init__(self, node):
        super(CaptureAndCalibrateSM, self).__init__()
        self.name = 'Capture and Calibrate Hand-Eye'
        self.node = node

        # Parámetros del comportamiento
        self.add_parameter('total_poses', 20)
        self.add_parameter('camera_type', 'realsense')
        self.add_parameter('base_frame', 'base_link')
        self.add_parameter('tool_frame', 'tool0')
        self.add_parameter('eye_in_hand', False)
        self.add_parameter('calibration_file_name', 'hand_eye_calibration.ini')
        
        # Parámetros de MoveIt para UR5e
        self.add_parameter('moveit_launch_file', 'ur_moveit.launch.py')
        self.add_parameter('robot_name', 'ur5e')
        self.add_parameter('moveit_config_package', 'ur_moveit_config')
        self.add_parameter('robot_ip', '192.168.1.101')
        self.add_parameter('use_fake_hardware', False)
        
        # RUTAS UNIFICADAS - Todo en /home/drims/drims_ws/calibrations
        base_calib_path = '/home/drims/drims_ws/calibrations'
        self.add_parameter('pictures_folder', f'{base_calib_path}/extrinsic_calibration/pictures')
        self.add_parameter('robot_poses_folder', f'{base_calib_path}/extrinsic_calibration/robot_poses')
        self.add_parameter('output_folder', base_calib_path)

        # Inicializar estados
        LaunchMoveItState.initialize_ros(node)
        TakePoseAndPictureState.initialize_ros(node)
        OfflineFindCharucoState.initialize_ros(node)
        ComputeCalibState.initialize_ros(node)
        OperatableStateMachine.initialize_ros(node)
        Logger.initialize(node)

        # [MANUAL_INIT]
        # [/MANUAL_INIT]

    def create(self):
        _state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
        
        # Variables para pasar datos entre estados
        _state_machine.userdata.base_h_tool_accumulated = None
        _state_machine.userdata.camera_h_charuco_accumulated = None

        # [MANUAL_CREATE]
        # [/MANUAL_CREATE]

        with _state_machine:
            # ESTADO 1: Lanzar MoveIt
            OperatableStateMachine.add('Launch_MoveIt',
                LaunchMoveItState(
                    moveit_launch_file=self.moveit_launch_file,
                    robot_name=self.robot_name,
                    moveit_config_package=self.moveit_config_package,
                    robot_ip=self.robot_ip,
                    use_fake_hardware=self.use_fake_hardware
                ),
                transitions={'done': 'Capture_Poses', 'failed': 'failed'},
                autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

            # ESTADO 2: Capturar poses (tú con ESPACIO)
            OperatableStateMachine.add('Capture_Poses',
                TakePoseAndPictureState(
                    total_poses=self.total_poses,
                    camera_type=self.camera_type,
                    base_frame=self.base_frame,
                    tool_frame=self.tool_frame,
                    pictures_folder=self.pictures_folder,
                    robot_poses_folder=self.robot_poses_folder,
                    output_folder=self.output_folder,
                    auto_capture=False
                ),
                transitions={'done': 'Process_Offline', 'failed': 'failed'},
                autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

            # ESTADO 3: Procesar offline con charuco_hand_eye
            OperatableStateMachine.add('Process_Offline',
                OfflineFindCharucoState(
                    pictures_folder=self.pictures_folder,
                    robot_poses_folder=self.robot_poses_folder,
                    eye_in_hand=self.eye_in_hand
                ),
                transitions={'completed': 'Compute_Calibration', 'failed': 'failed'},
                autonomy={'completed': Autonomy.Off, 'failed': Autonomy.Off},
                remapping={
                    'base_h_tool_accumulated': 'base_h_tool_accumulated',
                    'camera_h_charuco_accumulated': 'camera_h_charuco_accumulated'
                })

            # ESTADO 4: Calcular calibración con VISP
            OperatableStateMachine.add('Compute_Calibration',
                ComputeCalibState(
                    eye_in_hand_mode=self.eye_in_hand,
                    calibration_file_name=self.calibration_file_name,
                    customize_file=False,
                    launch_visp=True,
                    output_folder=self.output_folder
                ),
                transitions={'finish': 'finished', 'failed': 'failed'},
                autonomy={'finish': Autonomy.Off, 'failed': Autonomy.Off},
                remapping={
                    'base_h_tool': 'base_h_tool_accumulated',
                    'camera_h_charuco': 'camera_h_charuco_accumulated'
                })

        return _state_machine

    # [MANUAL_FUNC]
    # [/MANUAL_FUNC]
