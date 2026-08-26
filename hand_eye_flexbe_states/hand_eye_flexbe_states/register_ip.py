#!/usr/bin/env python3
import re
from flexbe_core import EventState, Logger

class RegisterIPState(EventState):
    """
    FlexBE State to register a new Robot IP into the manifest XML.

    -- new_robot_ip   str     IP address to register.

    <= done                   Successfully registered.
    <= failed                 Failed or invalid IP format.
    """
    def __init__(self, new_robot_ip='192.168.1.101'):
        super().__init__(outcomes=['done', 'failed'])
        self.new_robot_ip = new_robot_ip

    def execute(self, userdata):
        return 'done'

    def on_enter(self, userdata):
        try:
            ip_val = str(self.new_robot_ip).strip()
            if re.match(r'^\d{1,3}\.\d{1,3}\.\d{1,3}\.\d{1,3}$', ip_val):
                from hand_eye_flexbe_behaviors.manifest_utils import register_robot_ip_in_manifest
                register_robot_ip_in_manifest(ip_val)
                Logger.loginfo(f"✅ Successfully registered IP {ip_val} into FlexBE manifest!")
            else:
                Logger.logerr(f"❌ Invalid IP format: '{ip_val}'. Must be IPv4 (e.g. 192.168.1.105)")
        except Exception as e:
            Logger.logerr(f"❌ Failed to register IP: {e}")
