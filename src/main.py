import copy

import Scenario_
import src.utils_ as utils_
import opt
from log import timestamp, log_process_info, log_process_critical, log_process_debug, set_console_log_level
import logging

def main():
    score = 0
    set_console_log_level(logging.INFO)
    log_process_info('='*20 + ' New Test ' + '='*20)

    # fuzz
    utils_.restart_carla()
    for index, file in enumerate(utils_.get_seed(opt.test_dir)):
        if index < 2: continue
        seed_param = utils_.get_param(file)
        map_name = utils_.get_map_name(file)
        seed_param['map'] = map_name
        success_param = None
        test_param = seed_param

        log_process_info('select seed file: ' + file)
        dcycle_cnt = 0
        # INFO: dcount mean deep search, bcount mean broad search 
        while dcycle_cnt < opt.dcount:
            bscenario_param_list = []
            bcycle_cnt = 0
            if success_param: test_param = copy.deepcopy(success_param)
            while bcycle_cnt < opt.bcount:
                log_process_info('='*20 + f' Deep Cycle: {dcycle_cnt} Broad Cycle: {bcycle_cnt} ' + '='*20)
                score = 0
                is_collision = False
                params = {}
                try:
                    if opt.debug:
                        score, is_collision, params = Scenario_.make_and_run(test_param)
                    else:
                        ret_val = Scenario_.process_run(test_param)
                        if not ret_val: raise ValueError("exec failed")
                        elif type(ret_val) == str:
                            log_process_critical(ret_val)
                            raise ValueError("exec failed")
                        else:
                            score, is_collision, params = ret_val[0], ret_val[1], ret_val[2]
                    
                except Exception as e:
                    utils_.restart_carla()
                    utils_.close_sumo()
                    continue
                if not opt.debug: utils_.restart_carla()
                if opt.sumo: utils_.close_sumo()
                
                if params.get('score'): del params['score']
                if params.get('is_collision'): del params['is_collision']
                params = {'score': score, 'is_collision': is_collision, **params}
                bscenario_param_list.append((params, score))
                utils_.save_param(params, f"{map_name}_{'platoon' if opt.v2x else 'single'}_{dcycle_cnt}_{bcycle_cnt}.yaml", timestamp)
                
                if is_collision: break
                bcycle_cnt += 1

            # INFO: if collision car is success, set test_scenario to mutate
            # INFO: else set the highest score scenario to test_scenario
            if is_collision: 
                success_param = copy.deepcopy(params)
            else:
                test_param = max(bscenario_param_list, key=lambda x: x[1])[0]
            
            if opt.debug:
                for ind, val in enumerate(bscenario_param_list):
                    log_process_debug(f'index: {ind}, score: {val[1]}')

            dcycle_cnt += 1

    
    
if __name__ == '__main__':
    main()