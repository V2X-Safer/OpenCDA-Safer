import os
from os.path import dirname, realpath, join

from numpy import record

test_dir = join(os.getcwd(), 'src', 'test_yaml')
seed_dir = join(os.getcwd(), 'opencda', 'scenario_testing', 'config_yaml')
# TODO
sumo_cfg = ''
data_dump = False
additional_recorder = False

dcount = 15
bcount = 15
apply_ml = True
sumo = True
carla = True if not sumo else False
record = True
town = 'Town06'
record_file = f"{town}_{'sumo' if sumo else 'carla'}.log"
version = '0.9.12'




log_level = {
	'info': True,
	'warning': True,
	'error': True
}
debug = False


min_distance = 200
max_distance = 500
