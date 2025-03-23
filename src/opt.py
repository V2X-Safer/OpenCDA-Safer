import os
from os.path import dirname, realpath, join
import sys

from numpy import record



# INFO: fuzz setting
dcount = 5
bcount = 5
spawn_min_distance = 200
spawn_max_distance = 500
vehicle_min_distance = 3
vehicle_max_distance = 50
acc_threshold = 5
brake_threshold = 3
angular_threshold = 0.5
mutate_strategy = 'weather'


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
