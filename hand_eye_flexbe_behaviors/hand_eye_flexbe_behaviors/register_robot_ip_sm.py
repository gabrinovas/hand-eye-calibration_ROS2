#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, Logger
from hand_eye_flexbe_states.register_ip import RegisterIPState
# [MANUAL_IMPORT]
# [/MANUAL_IMPORT]


'''
Created on Thu Mar 05 2026
@author: Gabriel Novas
'''
class RegisterRobotIPSM(Behavior):
    """
    Utility Behavior to register a new Robot IP address into the FlexBE manifest.
    """
    
    def __init__(self, node):
        super(RegisterRobotIPSM, self).__init__()
        self.name = 'Register Robot IP'
        self.node = node

        # Behavior parameters
        self.add_parameter('new_robot_ip', '192.168.1.101')

        # Initialize states
        RegisterIPState.initialize_ros(node)
        OperatableStateMachine.initialize_ros(node)
        Logger.initialize(node)

        # [MANUAL_INIT]
        # [/MANUAL_INIT]

    def create(self):
        # x:30 y:50, x:130 y:50
        _state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

        # [MANUAL_CREATE]
        # [/MANUAL_CREATE]

        with _state_machine:
            # STATE 1: Register IP into manifest
            OperatableStateMachine.add('Register_IP_State',
                RegisterIPState(
                    new_robot_ip=self.new_robot_ip
                ),
                transitions={'done': 'finished', 'failed': 'failed'},
                autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

        return _state_machine

    # [MANUAL_FUNC]
    # [/MANUAL_FUNC]
