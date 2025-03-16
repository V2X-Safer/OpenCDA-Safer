import Scenario
import test
import src.utils_ as utils_
import opt
import scenario_runner


def main():
    score = 0

    for index, file in enumerate(utils_.get_seed()):
        seed_param = utils_.get_param(file)
        success_scenario = None

        # TODO
        test_scenario: Scenario.scenario = Scenario.scenario(seed_param)
        dcycle_cnt = 0

        # INFO: dcount mean deep search, bcount mean broad search 
        while dcycle_cnt < opt.dcount:
            if success_scenario: test_scenario: Scenario = success_scenario
            bscenario_list = []
            bcycle_cnt = 0
            while bcycle_cnt < opt.bcount:
                # TODO
                test_scenario.mutate()

                # TODO 
                score, is_success = test_scenario.run()

                bscenario_list.append((test_scenario, score))

                # INFO: if crack car is success, set test_scenario to mutate
                # INFO: else set the lowest score scenario to test_scenario
                if is_success: success_scenario = test_scenario
                else:
                    test_scenario = min(bscenario_list, key=lambda x: x[1])[0]
                
                if opt.debug:
                    for ind, val in enumerate(bscenario_list):
                        print(f'index: {ind}, score: {val[1]}')
                
                bcycle_cnt += 1

            dcycle_cnt += 1

    
    
if __name__ == '__main__':
    main()