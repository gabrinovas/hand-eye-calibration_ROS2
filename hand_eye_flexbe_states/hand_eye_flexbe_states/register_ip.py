#!/usr/bin/env python3
import re
from flexbe_core import EventState, Logger

class RegisterIPState(EventState):
    """
    FlexBE State to register a new Robot IP into the manifest XML.

    -- get_ip   callable/str   Dynamic getter for IP address to register.

    <= done                   Successfully registered.
    <= failed                 Failed or invalid IP format.
    """
    def __init__(self, get_ip=None):
        super().__init__(outcomes=['done', 'failed'])
        if callable(get_ip):
            self._get_ip_fn = get_ip
        else:
            self._get_ip_fn = lambda: get_ip

    def execute(self, userdata):
        return 'done'

    def on_enter(self, userdata):
        try:
            raw_ip = self._get_ip_fn()
            ip_val = str(raw_ip).strip()
            if re.match(r'^\d{1,3}\.\d{1,3}\.\d{1,3}\.\d{1,3}$', ip_val):
                from hand_eye_flexbe_behaviors.manifest_utils import register_robot_ip_in_manifest
                register_robot_ip_in_manifest(ip_val)
                Logger.loginfo(f"✅ Registered IP '{ip_val}' into manifest XML!")
                Logger.loginfo("💡 To refresh the dropdown in FlexBE App: Click 'Force Discover' under Configuration tab.")
            else:
                Logger.logerr(f"❌ Invalid IP format: '{ip_val}'. Must be IPv4 (e.g. 192.168.1.105)")
        except Exception as e:
            Logger.logerr(f"❌ Failed to register IP: {e}")
