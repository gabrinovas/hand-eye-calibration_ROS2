import os
import xml.etree.ElementTree as ET

def get_all_manifest_paths():
    paths = set()

    # 1. Relative path relative to this script location (works inside any workspace/docker structure)
    script_dir = os.path.dirname(os.path.abspath(__file__))
    relative_manifest = os.path.abspath(os.path.join(script_dir, '..', 'manifest', 'capture_and_calibrate.xml'))
    if os.path.exists(relative_manifest):
        paths.add(relative_manifest)

    # 2. Dynamic lookup via ROS 2 package indexing (ament_index)
    try:
        from ament_index_python.packages import get_package_prefix, get_package_share_directory
        try:
            prefix = get_package_prefix('hand_eye_flexbe_behaviors')
            for root_dir, _, files in os.walk(prefix):
                if 'capture_and_calibrate.xml' in files:
                    paths.add(os.path.join(root_dir, 'capture_and_calibrate.xml'))
        except Exception:
            pass
        try:
            share = get_package_share_directory('hand_eye_flexbe_behaviors')
            for root_dir, _, files in os.walk(share):
                if 'capture_and_calibrate.xml' in files:
                    paths.add(os.path.join(root_dir, 'capture_and_calibrate.xml'))
        except Exception:
            pass
    except Exception:
        pass

    return list(paths)

def register_robot_ip_in_manifest(ip_address: str):
    """
    Appends a new IP address to the robot_ip enum parameter options in capture_and_calibrate.xml
    without overwriting existing IPs across all discovered manifest locations.
    Uses relative script paths and dynamic ROS 2 ament indexing.
    """
    if not ip_address or ip_address in ["other", "otra", ""]:
        return

    manifest_paths = get_all_manifest_paths()
    for manifest_path in manifest_paths:
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
                        param.append(new_opt)

                        tree.write(manifest_path, encoding='UTF-8', xml_declaration=True)
                        print(f"✅ Added new IP '{ip_address}' to manifest options in {manifest_path}")
        except Exception as e:
            print(f"⚠️ Failed updating robot_ip in manifest {manifest_path}: {e}")

if __name__ == "__main__":
    import sys
    if len(sys.argv) > 1:
        register_robot_ip_in_manifest(sys.argv[1])
