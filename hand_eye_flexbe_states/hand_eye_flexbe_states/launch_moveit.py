#!/usr/bin/env python3
"""
FlexBE state to launch MoveIt automatically for UR5e.
"""

from flexbe_core import EventState, Logger
import subprocess
import time
import os
import psutil
from ament_index_python.packages import get_package_share_directory

class LaunchMoveItState(EventState):
    """
    Launches MoveIt with GUI for UR5e.
    
    -- moveit_launch_file    string   MoveIt launch file
    -- robot_name            string   Robot name (ur5e)
    -- moveit_config_package string   Configuration package
    -- robot_ip              string   UR robot IP
    -- use_fake_hardware     bool     Use simulation or real robot
    
    <= done                     MoveIt launched (even if not 100% ready)
    <= failed                   Error during launch
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
        """Launch MoveIt when starting the state"""
        if self.launch_attempted:
            return
            
        self.launch_attempted = True
        
        try:
            # Quick check: if MoveIt nodes exist, do not launch
            if self._is_moveit_basically_running():
                Logger.loginfo("✅ MoveIt is already running")
                return
            
            Logger.loginfo("="*60)
            Logger.loginfo("🚀 Launching MoveIt for UR5e...")
            Logger.loginfo("="*60)
            
            # Simplified command
            cmd = [
                'ros2', 'launch',
                self.moveit_config_package,
                self.moveit_launch_file,
                f'ur_type:={self.robot_name}',
                f'robot_ip:={self.robot_ip}',
                f'use_fake_hardware:={str(self.use_fake_hardware).lower()}',
                'launch_rviz:=true'
            ]
            
            Logger.loginfo(f"📋 Command: {' '.join(cmd)}")
            
            # Environment variables
            env = os.environ.copy()
            env['DISPLAY'] = ':0'
            
            # Launch in separate process
            self.process = subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                start_new_session=True,
                env=env
            )
            
            # Wait ONLY 3 seconds for basic startup
            Logger.loginfo("⏳ Waiting 3 seconds for MoveIt to start...")
            time.sleep(3)
            
            Logger.loginfo("✅ MoveIt launched, continuing with capture...")
            
        except Exception as e:
            Logger.logerr(f"❌ Error launching MoveIt: {str(e)}")
    
    def _is_moveit_basically_running(self):
        """Quick check: only looks if move_group node exists"""
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
        """Always return done to continue with capture"""
        return 'done'
    
    def on_stop(self):
        """Do nothing, MoveIt keeps running"""
        pass
