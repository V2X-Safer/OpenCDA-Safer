from doctest import debug
import os
import glob
from omegaconf import OmegaConf
import opt
from pprint import PrettyPrinter

pprint = PrettyPrinter(indent=opt.debug_indent, width=opt.debug_width, depth=opt.debug_depth).pprint


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

def merge_dict(dict1: dict, dict2: dict):
   return OmegaConf.merge(dict1, dict2) 


def get_seed_dir():
    return os.path.join(os.getcwd(), 'opencda', 'scenario_testing', 'config_yaml')

def get_seed():
    file_glob = glob.glob(os.path.join(get_seed_dir() ,'/*.yaml'))
    return [open(file) for file in file_glob]

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
