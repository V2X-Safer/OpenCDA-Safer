import os
from os.path import dirname, realpath, join

test_dir = join(os.getcwd(), 'src', 'test_yaml')
seed_dir = join(os.getcwd(), 'opencda', 'scenario_testing', 'config_yaml')

dcount = 15
bcount = 15
log_level = {
	'info': True,
	'warning': True,
	'error': True
}
debug = False


min_distance = 200
max_distance = 500
