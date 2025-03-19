from ctypes import util

from torch import seed
from src.Scenario_ import Scenario
import src.utils_ as utils_
import opt


def main():
    score = 0

    for index, file in enumerate(utils_.get_seed()):
        seed_param = utils_.get_param(file)
        map_name = utils_.get_map_name(file)
        seed_param['map'] = map_name
        success_param = None

        print('select seed file: ', file)
        test_scenario: Scenario = Scenario(seed_param)
        dcycle_cnt = 0

        # INFO: dcount mean deep search, bcount mean broad search 
        while dcycle_cnt < opt.dcount:
            if success_param: test_scenario: Scenario = Scenario(success_param)
            bscenario_list = []
            bcycle_cnt = 0
            while bcycle_cnt < opt.bcount:
                # TODO
                test_scenario.mutate()

                # TODO 
                score, is_success = test_scenario.run()

                bscenario_param_list.append((test_scenario.__dict__(), score))

                # INFO: if crack car is success, set test_scenario to mutate
                # INFO: else set the lowest score scenario to test_scenario
                if is_success: success_param = test_scenario.__dict__()
                else:
                    test_param = min(bscenario_list, key=lambda x: x[1])[0]
                    test_scenario
                
                if opt.debug:
                    for ind, val in enumerate(bscenario_list):
                        print(f'index: {ind}, score: {val[1]}')
                
                bcycle_cnt += 1

            dcycle_cnt += 1

    
    
if __name__ == '__main__':
    main()