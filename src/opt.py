import os
from os.path import dirname, realpath, join
import sys


# INFO: fuzz setting
dcount = 5
bcount = 5
near_distance = 20
spawn_min_distance = 200
spawn_max_distance = 500
vehicle_min_distance = 3
vehicle_max_distance = 50
acc_threshold = 5
brake_threshold = 3
angular_threshold = 0.5
mutate_strategy = 'random'
# mutate_world_strategy = ['weather', 'actor', 'traffic']
mutate_world_strategy = ['actor']
mutate_param_strategy = ['noise', 'platoon']
all_strategy = mutate_param_strategy + mutate_world_strategy
walker_strategy = 'random'
walker_speed_min = 1
walker_speed_max = 2
platoon_joined_penalty = 10

inter_gap_min = 0.5
inter_gap_max = 2
open_gap_min = 1
open_gap_max = 3
warm_up_speed_min = 20
warm_up_speed_max = 40  # 添加最大预热速度
communication_range_min = 10  # 添加最小通信范围
communication_range_max = 50

# 位置噪声参数(米)
loc_noise_min = 0.0  # 最小位置噪声
loc_noise_max = 0.5  # 最大位置噪声(0.5米是较为合理的GPS误差)

# 车辆控制参数
throttle_min = 0.2   # 最小油门控制值
throttle_max = 0.8   # 最大油门控制值
brake_min = 0.0      # 最小刹车控制值 
brake_max = 1.0      # 最大刹车控制值
steer_min = -0.8     # 最小转向控制值
steer_max = 0.8      # 最大转向控制值
hand_brake_prob = 0.05  # 使用手刹的概率
reverse_prob = 0.02     # 使用倒车的概率

# 天气参数范围
cloudiness_min = 0    # 最小云量
cloudiness_max = 100  # 最大云量
precipitation_min = 0    # 最小降水量 
precipitation_max = 100  # 最大降水量
precipitation_deposits_min = 0    # 最小积水量
precipitation_deposits_max = 100  # 最大积水量
wind_intensity_min = 0    # 最小风强度
wind_intensity_max = 100  # 最大风强度
fog_density_min = 0       # 最小雾密度
fog_density_max = 100     # 最大雾密度
fog_distance_min = 0      # 最小雾距离
fog_distance_max = 100    # 最大雾距离
wetness_min = 0           # 最小湿度
wetness_max = 100         # 最大湿度
sun_azimuth_angle_min = -180  # 最小太阳方位角
sun_azimuth_angle_max = 180   # 最大太阳方位角
sun_altitude_angle_min = -90  # 最小太阳高度角
sun_altitude_angle_max = 90   # 最大太阳高度角

# 朝向噪声参数(度)
yaw_noise_min = 0.0  # 最小朝向噪声
yaw_noise_max = 2.0  # 最大朝向噪声(2度是较为合理的方向传感器误差)

# 速度噪声参数(米/秒)
speed_noise_min = 0.0  # 最小速度噪声
speed_noise_max = 0.5  # 最大速度噪声(0.5米/秒是较为合理的速度测量误差)

# 通信时延参数(秒)
lag_min = 0.0  # 最小时延
lag_max = 0.3  # 最大时延(300毫秒是V2X通信中较为常见的延迟范围)

# LiDAR噪声参数（米）
lidar_noise_min = 0.01  # 高精度LiDAR的最小噪声
lidar_noise_max = 0.5   # 普通LiDAR在恶劣天气下的噪声

# 定位系统高度噪声（米）
alt_noise_min = 0.05  # 精确高度测量最小噪声
alt_noise_max = 2.0   # 普通GPS高度误差

# 定位系统纬度/经度噪声（米）
# 注意：当前使用的sys.maxsize太大，建议使用更合理的值
lat_noise_min = 0.01  # 高精度RTK-GPS最小噪声
lat_noise_max = 5.0   # 普通GPS在城市峡谷中的噪声

lon_noise_min = 0.01  # 高精度RTK-GPS最小噪声
lon_noise_max = 5.0   # 普通GPS在城市峡谷中的噪声

# INFO: misc
carla_version = '0.9.12'
carla_path = '/home/test/V2X/OpenCDA/carla_0_9_12/CarlaUE4/Binaries/Linux/CarlaUE4-Linux-Shipping'
maxsize = sys.maxsize
log_level = {
	'info': True,
	'warning': True,
	'error': True
}
debug = True
debug_indent = 4
debug_width = 100
debug_depth = 2
debug_line = '-' * 100
carla_port = 2000



# INFO: map setting
map = 'Town06'
town = map
map_helper = None


# INFO: scenario setting
apply_ml = True
xodr_file = None
v2x = False
application = ['single']
rsu = False


# INFO: file directory
test_dir = join(os.getcwd(), 'src', 'test_yaml')
seed_dir = join(os.getcwd(), 'opencda', 'scenario_testing', 'config_yaml')
sumo_dir = join(os.getcwd(), 'opencda', 'assets', map)
xodr_dir = join(os.getcwd(), 'opencda', 'assets', map)
picture_dir = join(os.getcwd(), 'src', 'log', 'view')
param_dir = join(os.getcwd(), 'src', 'log', 'param')


# INFO: traffic flow setting
sumo = False
sumo_cfg = None


# INFO: data collection setting
record = True
data_dump = False
additional_recorder = False
record_file = f"{map}_{'cosim' if sumo else 'carla'}.log"
picture_save_file = 'location.jpg'
