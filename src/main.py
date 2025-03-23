import copy
from src.Scenario_ import Scenario
import src.utils_ as utils_
import opt


def main():
    score = 0

    for index, file in enumerate(utils_.get_seed(opt.test_dir)):
        if index < 2: continue
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
                test_scenario = Scenario(success_param if success_param else test_param)
                test_scenario.mutate()

                score, is_success = test_scenario.run()

                bcycle_cnt += 1

                param = {
                    'score': score,
                    'is_success': is_success,
                }
                param.update(copy.deepcopy(test_scenario.get_raw_param()))

                bscenario_param_list.append((param, score))
                utils_.save_param(param, f'{map_name}_{index}_{dcycle_cnt}_{bcycle_cnt}.yaml')
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