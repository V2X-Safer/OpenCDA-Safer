import copy
import datetime
import signal
import trace
import traceback
from src.Scenario_ import Scenario
import src.utils_ as utils_
import opt

def handler(signum, frame):
    raise Exception("HANG")

def main():
    score = 0
    signal.signal(signal.SIGALRM, handler)
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
        # INFO: dcount mean deep search, bcount mean broad search 
        while dcycle_cnt < opt.dcount:
            bscenario_param_list = []
            bcycle_cnt = 0
            is_success = False
            while bcycle_cnt < opt.bcount:
                # signal.alarm(10*60)
                try:
                    test_scenario = Scenario(success_param if success_param else test_param)
                    test_scenario.mutate()

                    score, is_success = test_scenario.run()
                except Exception as e:
                    if e.args[0] == 'HANG':
                        print("HANG")
                        bcycle_cnt -= 1
                    else:
                        traceback.print_exc()

                bcycle_cnt += 1
                # signal.alarm(0)

                param = {
                    'score': score,
                    'is_success': is_success,
                }
                param.update(copy.deepcopy(test_scenario.get_raw_param()))

                bscenario_param_list.append((param, score))
                utils_.save_param(param, f'{map_name}_{index}_{dcycle_cnt}_{bcycle_cnt}.yaml', timestamp)
            # INFO: if crack car is success, set test_scenario to mutate
            # INFO: else set the lowest score scenario to test_scenario
            if is_success: success_param = test_scenario.get_raw_param()
            else:
                test_param = max(bscenario_param_list, key=lambda x: x[1])[0]
            
            if opt.debug:
                for ind, val in enumerate(bscenario_param_list):
                    print(f'index: {ind}, score: {val[1]}')

            dcycle_cnt += 1

    
    
if __name__ == '__main__':
    main()