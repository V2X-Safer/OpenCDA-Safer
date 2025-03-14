import os
import glob
from omegaconf import OmegaConf
import opt


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
