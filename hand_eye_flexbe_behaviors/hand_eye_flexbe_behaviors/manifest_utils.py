import os
import xml.etree.ElementTree as ET

def register_robot_ip_in_manifest(ip_address: str):
    """
    Appends a new IP address to the robot_ip enum parameter options in capture_and_calibrate.xml
    without overwriting existing IPs.
    """
    if not ip_address or ip_address in ["other", "otra", ""]:
        return

    possible_manifest_paths = []
    try:
        from ament_index_python.packages import get_package_share_directory
        share_dir = get_package_share_directory('hand_eye_flexbe_behaviors')
        possible_manifest_paths.append(os.path.join(share_dir, 'manifest', 'capture_and_calibrate.xml'))
    except Exception:
        pass

    repo_manifest = os.path.expanduser('~/APERTA/APERTA-Repos/camera-calibration/hand-eye-calibration_ROS2/hand_eye_flexbe_behaviors/manifest/capture_and_calibrate.xml')
    if os.path.exists(repo_manifest):
        possible_manifest_paths.append(repo_manifest)

    for manifest_path in possible_manifest_paths:
        if not os.path.exists(manifest_path):
            continue
        try:
            tree = ET.parse(manifest_path)
            root = tree.getroot()
            params = root.find('params')
            if params is None:
                continue

            for param in params.findall('param'):
                if param.get('name') == 'robot_ip':
                    existing_vals = [opt.get('value') for opt in param.findall('option')]
                    if ip_address not in existing_vals:
                        new_opt = ET.Element('option')
                        new_opt.set('value', ip_address)
                        
                        other_opt = None
                        for opt in list(param.findall('option')):
                            if opt.get('value') in ['other', 'otra']:
                                other_opt = opt
                                break
                        
                        if other_opt is not None:
                            idx = list(param).index(other_opt)
                            param.insert(idx, new_opt)
                        else:
                            param.append(new_opt)

                        tree.write(manifest_path, encoding='UTF-8', xml_declaration=True)
                        print(f"✅ Added new IP '{ip_address}' to manifest options in {manifest_path}")
        except Exception as e:
            print(f"⚠️ Failed updating robot_ip in manifest {manifest_path}: {e}")

if __name__ == "__main__":
    import sys
    if len(sys.argv) > 1:
        register_robot_ip_in_manifest(sys.argv[1])
