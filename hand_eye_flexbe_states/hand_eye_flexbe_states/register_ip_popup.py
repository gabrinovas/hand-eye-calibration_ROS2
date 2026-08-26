#!/usr/bin/env python3
import os
import re
import tkinter as tk
from tkinter import messagebox
from flexbe_core import EventState, Logger

class RegisterIPPopupState(EventState):
    """
    FlexBE State that optionally opens a GUI popup window at start to register a new Robot IP.
    
    -- register_new_ip_popup  bool    If True, opens GUI popup. If False, proceeds immediately.
    -- robot_ip               str     Default/Current IP selected from manifest.
    
    #> new_robot_ip           str     Final IP address to use (registered or selected).
    <= done                           State finished successfully.
    <= failed                         Error/Canceled.
    """
    def __init__(self, register_new_ip_popup=False, robot_ip='192.168.1.101'):
        super().__init__(
            outcomes=['done', 'failed'],
            output_keys=['new_robot_ip']
        )
        self.register_new_ip_popup = register_new_ip_popup
        self.robot_ip = robot_ip
        self.registered_ip = robot_ip

    def on_enter(self, userdata):
        if not self.register_new_ip_popup:
            userdata.new_robot_ip = self.robot_ip
            return 'done'

        try:
            self._show_gui()
            userdata.new_robot_ip = self.registered_ip
            return 'done'
        except Exception as e:
            Logger.logwarn(f"⚠️ GUI popup failed: {e}")
            userdata.new_robot_ip = self.robot_ip
            return 'done'

    def _show_gui(self):
        root = tk.Tk()
        root.title("Register New Robot IP")
        root.geometry("380x200")
        root.resizable(False, False)
        
        # Center window on screen
        root.update_idletasks()
        width = root.winfo_width()
        height = root.winfo_height()
        x = (root.winfo_screenwidth() // 2) - (width // 2)
        y = (root.winfo_screenheight() // 2) - (height // 2)
        root.geometry(f'+{x}+{y}')

        tk.Label(root, text="Register New Robot IP Address", font=("Helvetica", 12, "bold")).pack(pady=10)
        tk.Label(root, text="Enter the IP address of your robot:", font=("Helvetica", 10)).pack(pady=2)

        ip_var = tk.StringVar(value=self.robot_ip)
        entry = tk.Entry(root, textvariable=ip_var, font=("Consolas", 12), width=22, justify="center")
        entry.pack(pady=10)
        entry.focus_set()

        def save_and_close():
            ip_val = ip_var.get().strip()
            if not re.match(r'^\d{1,3}\.\d{1,3}\.\d{1,3}\.\d{1,3}$', ip_val):
                messagebox.showerror("Invalid IP", "Please enter a valid IP address (e.g., 192.168.1.102)")
                return
            
            try:
                from hand_eye_flexbe_behaviors.manifest_utils import register_robot_ip_in_manifest
                register_robot_ip_in_manifest(ip_val)
            except Exception as e:
                print(f"Error registering IP: {e}")

            self.registered_ip = ip_val
            messagebox.showinfo("Success", f"IP {ip_val} registered successfully into manifest!")
            root.destroy()

        def cancel_and_close():
            root.destroy()

        btn_frame = tk.Frame(root)
        btn_frame.pack(pady=10)

        tk.Button(btn_frame, text="💾 Save IP", command=save_and_close, bg="#4CAF50", fg="white", font=("Helvetica", 10, "bold"), width=12).pack(side=tk.LEFT, padx=5)
        tk.Button(btn_frame, text="❌ Cancel", command=cancel_and_close, bg="#f44336", fg="white", font=("Helvetica", 10, "bold"), width=12).pack(side=tk.LEFT, padx=5)

        root.mainloop()
