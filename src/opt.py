import os
from os.path import dirname, realpath, join

from numpy import record


# INFO: map setting
map = '2lane_freeway_simplified'
town = map
map_helper = None


# INFO: scenario setting
apply_ml = True
xodr_file = None
v2x = False
application = ['single']


# INFO: file directory
test_dir = join(os.getcwd(), 'src', 'test_yaml')
seed_dir = join(os.getcwd(), 'opencda', 'scenario_testing', 'config_yaml')
sumo_dir = join(os.getcwd(), 'opencda', 'assets', map)
xodr_dir = join(os.getcwd(), 'opencda', 'assets', map)


# INFO: traffic flow setting
sumo = False
sumo_cfg = None


# INFO: data collection setting
record = True
data_dump = False
additional_recorder = False
record_file = f"{town}_{'sumo' if sumo else 'carla'}.log"


# INFO: fuzz setting
dcount = 15
bcount = 15
min_distance = 200
max_distance = 500


# INFO: misc
carla_version = '0.9.12'
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
