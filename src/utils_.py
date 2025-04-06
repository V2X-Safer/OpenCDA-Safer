import os
import glob
from omegaconf import OmegaConf
import opt
import subprocess
import time
from log import *


def restart_carla():
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
        # sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # sock.settimeout(1)
        # port_in_use = sock.connect_ex(('localhost', carla_port)) == 0
        # sock.close()
        
        # Kill any existing Carla processes (including parent processes of zombies)
        log_process_debug('Restarting Carla...')
        if carla_pids  or has_defunct:
            log_process_debug(f"Found existing Carla processes or port {carla_port} in use")
            
            # Kill Carla processes
            for pid in carla_pids:
                log_process_debug(f"Killing Carla process with PID {pid}")
                try:
                    subprocess.run(["kill", "-9", str(pid)])
                except Exception as e:
                    log_process_critical(f"Error killing process {pid}: {e}")
            
            # If we had defunct processes, try to kill their parent processes
            if has_defunct:
                # Find and kill parent processes of zombies
                subprocess.run(["pkill", "-9", "-f", "CarlaUE4"])
                
            # Wait for processes to terminate and port to be released
            time.sleep(1)
            log_process_debug("Carla processes have been terminated")
            
            # Double-check if port is now free
            # sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            # sock.settimeout(1)
            # port_still_in_use = sock.connect_ex(('localhost', carla_port)) == 0
            # sock.close()
            
            # if port_still_in_use:
                # print(f"Warning: Port {carla_port} is still in use after killing processes")
        
        # Start Carla using subprocess instead of fork
        carla_cmd = [opt.carla_path, f"-carla-rpc-port={carla_port}"]
        carla_process = subprocess.Popen(carla_cmd)
        
        log_process_debug(f"Carla started with PID {carla_process.pid}")
        # Wait for Carla to initialize
        time.sleep(3)
        
        # Verify Carla is running and port is open
        # sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # sock.settimeout(1)
        # if sock.connect_ex(('localhost', carla_port)) == 0:
            # print(f"Carla is running on port {carla_port}")
            # sock.close()
        # else:
            # print(f"Warning: Carla may not have started correctly. Port {carla_port} is not open.")
            # sock.close()
            
    except Exception as e:
        log_process_critical(f"Error while managing Carla: {e}")

def save_param(param: dict, map_name: str, dcycle: int, bcycle: int, timestamp: str, app_type: str = 'single', sim_type: str = 'carla'):
    """
    将参数字典保存为YAML文件。
    
    Parameters
    ----------
    param : dict
        要保存的参数字典
    map_name : str
        地图名称
    dcycle : int
        深度循环计数
    bcycle : int
        广度循环计数
    timestamp : str
        时间戳
    app_type : str
        应用类型，如 'single' 或 'platoon'
    sim_type : str
        仿真类型，如 'carla' 或 'cosim'
    """
    try:
        folder_name = f"{map_name}_{app_type}_{sim_type}"
        dir_path = os.path.join(opt.param_dir, timestamp, folder_name)
        
        if not os.path.exists(dir_path):
            os.makedirs(dir_path)
        
        file_name = f"d{dcycle}_b{bcycle}.yaml"
        save_path = os.path.join(dir_path, file_name)
        
        param['current_time'] = timestamp
        
        conf = OmegaConf.create(param)
        OmegaConf.save(conf, save_path)
        
        log_process_info(f"successfully save param to {save_path}")
        
    except Exception as e:
        log_process_critical(f"保存参数到 {file_name} 时出错: {e}")
        
        try:
            import yaml
            os.makedirs(dir_path, exist_ok=True)
            with open(save_path, 'w') as file:
                yaml.dump(param, file, default_flow_style=False)
                log_process_debug(f"使用PyYAML将参数保存到 {save_path}")
        except Exception as backup_e:
            log_process_critical(f"备用保存方法也失败: {backup_e}")

            
def get_param(target_file: str, debug=False):
    # set default dir to test_yaml
    default_yaml = config_yaml = os.path.join(opt.test_dir, 'default.yaml')
    # config_yaml to test.yaml
    config_yaml = os.path.join(opt.test_dir, target_file)

    # TODO: add log
    if debug: pass

    default_dict = OmegaConf.load(default_yaml)
    scene_dict = OmegaConf.load(config_yaml)
    merged_dict = merge_dict(default_dict, scene_dict)
    merged_dict['map'] = get_map_name(target_file)
    return merged_dict

def get_map_name(target_file: str):
    """
    根据目标文件名确定地图名称。
    
    Parameters
    ----------
    target_file : str
        目标配置文件名
        
    Returns
    -------
    str
        对应的CARLA地图名称
    """
    # 基本CARLA城市地图
    if 'town05' in target_file.lower():
        return 'Town05'
    elif 'town06' in target_file.lower():
        return 'Town06'
    elif 'town01' in target_file.lower():
        return 'Town01'
    elif 'town02' in target_file.lower():
        return 'Town02'
    elif 'town03' in target_file.lower():
        return 'Town03'
    elif 'town04' in target_file.lower():
        return 'Town04'
    elif 'town07' in target_file.lower():
        return 'Town07'
    elif 'town10' in target_file.lower():
        return 'Town10HD'
    elif 'town11' in target_file.lower():
        return 'Town11'
    elif 'town12' in target_file.lower():
        return 'Town12'
        
    # 自定义地图
    elif '2lanefree' in target_file.lower():
        return '2lane_freeway_simplified'
    elif 'highway' in target_file.lower():
        return 'Highway'
    elif 'oval' in target_file.lower():
        return 'OvalTrack'
    elif 'junction' in target_file.lower():
        return 'ComplexJunction'
        
    # 默认地图
    else:
        log_process_debug(f"未能识别地图名称 '{target_file}'，使用默认地图 'Town06'")
        return 'Town06'

def get_vehicle_distance(vehicle1, vehicle2):
    return get_distance(vehicle1.get_transform(), vehicle2.get_transform())

def get_distance(transform1, transform2):
    return transform1.location.distance(transform2.location)


def merge_dict(dict1: dict, dict2: dict):
   return OmegaConf.merge(dict1, dict2) 


def get_seed(target_file_dir: str = opt.seed_dir):
    file_glob = glob.glob(target_file_dir + os.sep + '*.yaml')
    return [file for file in file_glob if not any(black_file in file for black_file in ['test.yaml', 'v2xp', 'default.yaml', 'openscenario_carla.yaml', 'cosim'])]

def get_xodr_path(xodr_file: str = '2lane_freeway_simplified.xodr'):
    return os.path.join(os.getcwd(),
                        'opencda',
                        'assets',
                        '2lane_freeway_simplified',
                        xodr_file)


def close_sumo():
    """
    检查SUMO是否正在运行，如果是则关闭它。
    处理所有SUMO相关进程，包括sumo-gui、sumo和任何相关的TraCI进程。
    """
    try:
        # 检查所有相关SUMO进程
        result = subprocess.run(["ps", "aux"], capture_output=True, text=True)
        sumo_processes = [line for line in result.stdout.splitlines() 
                          if any(x in line for x in ["sumo-gui", "sumo ", "traci"])]
        
        sumo_pids = []
        for line in sumo_processes:
            parts = line.split()
            if len(parts) > 1:
                try:
                    pid = int(parts[1])
                    sumo_pids.append(pid)
                except ValueError:
                    pass
        
        # 如果找到SUMO进程，则终止它们
        if sumo_pids:
            log_process_debug(f"发现正在运行的SUMO进程，正在关闭...")
            for pid in sumo_pids:
                log_process_debug(f"正在终止SUMO进程 (PID: {pid})")
                try:
                    subprocess.run(["kill", "-9", str(pid)])
                except Exception as e:
                    log_process_critical(f"终止进程 {pid} 时出错: {e}")
            
            # 等待进程完全终止
            time.sleep(1)
            log_process_debug("SUMO进程已终止")
            
            # 确保所有SUMO相关进程都被终止
            subprocess.run(["pkill", "-9", "-f", "sumo"])
            subprocess.run(["pkill", "-9", "-f", "traci"])
            
    except Exception as e:
        log_process_critical(f"检查和关闭SUMO时出错: {e}")
