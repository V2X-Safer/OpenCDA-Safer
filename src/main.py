import copy
import datetime
import traceback
import Scenario_
import src.utils_ as utils_
import opt

def handler(signum, frame):
    raise Exception("HANG")

def main():
    score = 0
    # signal.signal(signal.SIGALRM, handler)
    current_datetime = datetime.datetime.now()
    timestamp = current_datetime.strftime("%Y_%m_%d-%H_%M")
    for index, file in enumerate(utils_.get_seed(opt.seed_dir)):
        if index < 0: continue
        seed_param = utils_.get_param(file)
        map_name = utils_.get_map_name(file)
        seed_param['map'] = map_name
        success_param = None
        test_param = seed_param

        print('select seed file: ', file)
        dcycle_cnt = 0
        utils_.restart_carla()
        # INFO: dcount mean deep search, bcount mean broad search 
        while dcycle_cnt < opt.dcount:
            bscenario_param_list = []
            bcycle_cnt = 0
            is_success = False
            if success_param: test_param = success_param
            while bcycle_cnt < opt.bcount:
                # signal.alarm(10*60)
                try:
                    score, is_success, params = Scenario_.process_run(test_param)
                    
                except Exception as e:
                    if e.args[0] == 'exec failed':
                        print("HANG")
                    else:
                        traceback.print_exc()
                    continue
                utils_.restart_carla()
                
                if params.get('score'): del params['score']
                if params.get('is_success'): del params['is_success']
                params = {'score': score, 'is_success': is_success, **params}
                bscenario_param_list.append((params, score))
                utils_.save_param(params, f'{map_name}_{index}_{dcycle_cnt}_{bcycle_cnt}.yaml', timestamp)

                bcycle_cnt += 1

            # INFO: if crack car is success, set test_scenario to mutate
            # INFO: else set the lowest score scenario to test_scenario
            if is_success: success_param = copy.deepcopy(params)
            else:
                test_param = max(bscenario_param_list, key=lambda x: x[1])[0]
            
            if opt.debug:
                for ind, val in enumerate(bscenario_param_list):
                    print(f'index: {ind}, score: {val[1]}')

            dcycle_cnt += 1

    
    
if __name__ == '__main__':
    main()