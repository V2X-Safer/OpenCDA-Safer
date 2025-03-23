import os
import glob
from omegaconf import OmegaConf
import opt
from pprint import PrettyPrinter
import subprocess
import time
import socket

pprint = PrettyPrinter(indent=opt.debug_indent, width=opt.debug_width, depth=opt.debug_depth).pprint


def check_carla():
    """
    Check if Carla is running. If it is running, kill it; otherwise, start it.
    Handles defunct/zombie processes and ensures proper port availability.
    """
    try:
        carla_port = getattr(opt, 'carla_port', 2000)  # Default to 2000 if not specified
        
        # Check for any Carla processes (including defunct/zombie ones)
        result = subprocess.run(["ps", "aux"], capture_output=True, text=True)
        carla_lines = [line for line in result.stdout.splitlines() if "CarlaUE4" in line]
        carla_pids = []
        
        # Extract PIDs and check for defunct processes
        has_defunct = False
        for line in carla_lines:
            if "<defunct>" in line or "Z+" in line:
                has_defunct = True
            parts = line.split()
            if len(parts) > 1:
                try:
                    pid = int(parts[1])
                    carla_pids.append(pid)
                except ValueError:
                    pass
        
        # Check if the port is in use
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(1)
        port_in_use = sock.connect_ex(('localhost', carla_port)) == 0
        sock.close()
        
        # Kill any existing Carla processes (including parent processes of zombies)
        if carla_pids or port_in_use or has_defunct:
            print(f"Found existing Carla processes or port {carla_port} in use")
            
            # Kill Carla processes
            for pid in carla_pids:
                print(f"Killing Carla process with PID {pid}")
                try:
                    subprocess.run(["kill", "-9", str(pid)])
                except Exception as e:
                    print(f"Error killing process {pid}: {e}")
            
            # If we had defunct processes, try to kill their parent processes
            if has_defunct:
                # Find and kill parent processes of zombies
                subprocess.run(["pkill", "-9", "-f", "CarlaUE4"])
                
            # Wait for processes to terminate and port to be released
            time.sleep(1)
            print("Carla processes have been terminated")
            
            # Double-check if port is now free
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(1)
            port_still_in_use = sock.connect_ex(('localhost', carla_port)) == 0
            sock.close()
            
            if port_still_in_use:
                print(f"Warning: Port {carla_port} is still in use after killing processes")
        
        # Start Carla using subprocess instead of fork
        print("Starting Carla...")
        carla_cmd = [opt.carla_path, f"-carla-rpc-port={carla_port}"]
        carla_process = subprocess.Popen(carla_cmd, stderr=subprocess.DEVNULL, stdout=subprocess.DEVNULL)
        
        print(f"Carla started with PID {carla_process.pid}")
        # Wait for Carla to initialize
        time.sleep(3)
        
        # Verify Carla is running and port is open
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(1)
        if sock.connect_ex(('localhost', carla_port)) == 0:
            print(f"Carla is running on port {carla_port}")
            sock.close()
        else:
            print(f"Warning: Carla may not have started correctly. Port {carla_port} is not open.")
            sock.close()
            
    except Exception as e:
        print(f"Error while managing Carla: {e}")

def save_param(param: dict, file_name: str):
    """
    将参数字典保存为YAML文件。
    
    Parameters
    ----------
    param : dict
        要保存的参数字典
    save_path : str
        保存的文件路径，应以.yaml结尾
    """
    try:
        # 确保目标目录存在
        import os
        dir_path = opt.param_dir
        if (dir_path and not os.path.exists(dir_path)):
            os.makedirs(dir_path)
        
        # 处理文件扩展名
        if not file_name.endswith('.yaml'):
            file_name = f"{file_name}.yaml"
        
        # 将字典转换为OmegaConf并保存
        from omegaconf import OmegaConf
        conf = OmegaConf.create(param)
        save_path = os.path.join(dir_path, file_name)
        OmegaConf.save(conf, save_path)
        
        print(f"Parameters successfully saved to {file_name}")
        
    except Exception as e:
        print(f"Error saving parameters to {file_name}: {e}")
        
        # 尝试备用方法保存
        try:
            import yaml
            with open(file_name, 'w') as file:
                yaml.dump(param, file, default_flow_style=False)
            print(f"Parameters saved using PyYAML to {file_name}")
        except Exception as backup_e:
            print(f"Backup save method also failed: {backup_e}")

def get_param(target_file: str, debug=False):
    # set default dir to test_yaml
    default_yaml = config_yaml = os.path.join(opt.test_dir, 'default.yaml')
    # config_yaml to test.yaml
    config_yaml = os.path.join(opt.test_dir, target_file)

    # TODO: add log
    if debug: pass

    default_dict = OmegaConf.load(default_yaml)
    scene_dict = OmegaConf.load(config_yaml)
    return merge_dict(default_dict, scene_dict)

def get_map_name(target_file: str):
    if 'town05' in target_file:
        return 'Town05'
    elif 'town06' in target_file:
        return 'Town06'
    elif '2lanefree' in target_file:
        return '2lane_freeway_simplified'
    else:
        return 'Town06'

def get_vehicle_distance(vehicle1, vehicle2):
    return get_distance(vehicle1.get_transform(), vehicle2.get_transform())

def get_distance(transform1, transform2):
    return transform1.location.distance(transform2.location)


def merge_dict(dict1: dict, dict2: dict):
   return OmegaConf.merge(dict1, dict2) 


def get_seed(target_file_dir: str = opt.seed_dir):
    file_glob = glob.glob(target_file_dir + os.sep + '*.yaml')
    return [file for file in file_glob if not any(black_file in file for black_file in ['test.yaml', 'v2xp', 'default.yaml', 'openscenario_carla.yaml'])]

def get_xodr_path(xodr_file: str = '2lane_freeway_simplified.xodr'):
    return os.path.join(os.getcwd(),
                        'opencda',
                        'assets',
                        '2lane_freeway_simplified',
                        xodr_file)
    
def debug_hint(func):
    def inner(*args, **kwargs):
        if opt.debug: 
            pprint(opt.debug_line)
            res = func(*args, **kwargs)
            pprint(opt.debug_line)
            return res
    return inner 


@debug_hint
def debug_dict(dict_: dict, debug=False, exclude=[]):
    if not debug: return
    for key, value in dict_.items():
        if not key.startswith('__') and key not in exclude: 
            pprint(f'{key}: {value}')
