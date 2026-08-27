import os
import sys
import xml.etree.ElementTree as ET

def get_all_manifest_paths():
    paths = set()

    # 1. Search relative to this script location
    script_dir = os.path.dirname(os.path.abspath(__file__))
    curr = script_dir
    for _ in range(5):
        manifest_candidate = os.path.join(curr, 'manifest', 'capture_and_calibrate.xml')
        if os.path.exists(manifest_candidate):
            paths.add(os.path.abspath(manifest_candidate))
        curr = os.path.dirname(curr)

    # 2. Search ROS 2 AMENT_PREFIX_PATH if set
    ament_paths = os.environ.get('AMENT_PREFIX_PATH', '').split(os.path.pathsep)
    for p in ament_paths:
        if p and os.path.exists(p):
            for root_dir, _, files in os.walk(p):
                if 'capture_and_calibrate.xml' in files:
                    paths.add(os.path.join(root_dir, 'capture_and_calibrate.xml'))

    # 3. Search common workspace roots (both src AND install)
    search_roots = [
        os.path.expanduser('~/static/aperta_ws'),
        os.path.expanduser('~/APERTA'),
        os.path.expanduser('~/aperta_ws'),
    ]
    for s_root in search_roots:
        if os.path.exists(s_root):
            for root_dir, _, files in os.walk(s_root):
                if 'capture_and_calibrate.xml' in files:
                    paths.add(os.path.join(root_dir, 'capture_and_calibrate.xml'))

    return list(paths)

def register_robot_ip_in_manifest(ip_address: str):
    """
    Appends a new IP address to the robot_ip enum parameter options in capture_and_calibrate.xml
    without overwriting existing IPs across all discovered manifest locations (both src and install).
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
    if len(sys.argv) > 1:
        register_robot_ip_in_manifest(sys.argv[1])
