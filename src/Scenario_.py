import copy
import carla
import torch.multiprocessing as mp
import os
import random
import carla
from opencda.core.common.cav_world import CavWorld
from opencda.core.common.vehicle_manager import VehicleManager
from opencda.scenario_testing.utils import cosim_api, customized_map_api, sim_api
from typing import Union
from opencda.scenario_testing.utils.yaml_utils import add_current_time

from src import operation
from src.oracle_manager import OracleManager
import src.utils_ as utils_
import opt
from log import *

class Scenario:
    '''
    统一管理所有的manager 
    
    Parameters:
    -----------
    scenario_params : dict
        场景配置参数字典，包含场景测试所需的各种参数。
    
    Attributes:
    -----------
    scenario_manager : sim_api.ScenarioManager
        场景管理器实例，负责整个场景的创建、协调和生命周期管理。
        
    platoon_manager_list : list[platooning_manager.PlatooningManager]
        编队管理器列表，通过 create_platoon_manager 方法生成的编队管理对象集合。
        
    single_cav_list : list
        单车辆管理器列表，包含通过 create_vehicle_manager 创建的独立网联自动驾驶车辆管理器。
        
    traffic_manager : carla.TrafficManager, optional
        CARLA 交通管理器实例，仅在 opt.carla 为 True 时创建，用于控制背景交通流。
        
    bg_veh_list : list[carla.Vehicle], optional
        背景车辆列表，仅在启用 CARLA 交通时生成，包含场景中所有背景车辆实例。
        
    platoon_list : list[platooning_manager.PlatooningManager]
        编队管理器列表（可能与 platoon_manager_list 重复），用于集中管理所有车辆编队行为。
        
    evaluation_manager : EvaluationManager
        评估管理器实例，负责场景测试过程中的数据收集和性能评估。
        
    spectator : carla.Actor
        观察者视角实例，用于控制场景渲染的摄像机视角，默认设置为俯视视角。
    '''

    def __init__(self, scenario_params):
        self.init_opt(scenario_params)
        if opt.debug:
            scenario_params['world']['client_port'] = opt.carla_port
        
        self.raw_param = scenario_params
        scenario_params = add_current_time(scenario_params)
        self.scenario_manager = self.ScenarioManager(scenario_params)

        if opt.record:
            self.scenario_manager.client.start_recorder(
                opt.record_file,
                opt.additional_recorder)
        self.platoon_list = []
        self.rsu_list = []
        if opt.v2x:
            self.platoon_list = self.scenario_manager.create_platoon_manager(
                map_helper=opt.map_helper,
                data_dump=opt.data_dump)

        self.single_cav_list: list[VehicleManager] =  self.scenario_manager.create_vehicle_manager(
                application=opt.application,
                map_helper=opt.map_helper,
                data_dump=opt.data_dump)
        if opt.rsu:
            self.rsu_list = self.scenario_manager.create_rsu_manager(opt.data_dump)

        # create carla traffic flow
        if not opt.sumo:
            self.traffic_manager, self.bg_veh_list  = \
                self.scenario_manager.create_traffic_carla()

        self.oracle_manager = \
            OracleManager(self.scenario_manager.cav_world,
                                              script_name=opt.record_file,
                                              current_time=scenario_params['current_time'])
        self.spectator = self.scenario_manager.world.get_spectator()
        if opt.v2x:
            self.spectator_vehicle = self.platoon_list[0].vehicle_manager_list[1].vehicle
        else:
            self.spectator_vehicle = self.single_cav_list[0].vehicle
        self.operation = operation.Operation(self)
        self.init_mutator()


    def ScenarioManager(self, scenario_params) -> Union[sim_api.ScenarioManager, cosim_api.CoScenarioManager]:
        """
        Creates and returns the appropriate ScenarioManager based on configuration.
        
        Returns:
            Union[sim_api.ScenarioManager, cosim_api.CoScenarioManager]: 
                CoScenarioManager if V2X is enabled, otherwise ScenarioManager
        """

        manager_args = {
            "scenario_params": scenario_params,
            "apply_ml": opt.apply_ml,
            "carla_version": opt.carla_version,
            "town": opt.town,
            "xodr_path": opt.xodr_file
        }

        if opt.sumo:
            self.scenario_manager_cls = cosim_api.CoScenarioManager
        else: 
            self.scenario_manager_cls = sim_api.ScenarioManager
        
        if opt.sumo_cfg:
            manager_args["sumo_file_parent_path"] = opt.sumo_cfg
        if not opt.v2x:
            # platooning_manager won't create cav_world
            manager_args['cav_world'] = CavWorld(opt.apply_ml)

        return self.scenario_manager_cls(**manager_args)



    def run(self):
        ''' 运行场景 '''
        try: 
            while True:
                self.scenario_manager.cav_world.tick()
                self.scenario_manager.tick()
                transform = self.spectator_vehicle.get_transform()
                self.spectator.set_transform(
                    carla.Transform(transform.location +
                        carla.Location(z=80),
                        carla.Rotation(pitch=-90)))

                if opt.v2x:
                    for platoon in self.platoon_list:
                        platoon.update_information()
                        platoon.run_step()

                for i, single_cav in enumerate(self.single_cav_list):
                    # this function should be added in wrapper
                    if single_cav.v2x_manager.in_platoon():
                        self.single_cav_list.pop(i)
                    else:
                        single_cav.update_info()
                        control = single_cav.run_step()
                        single_cav.vehicle.apply_control(control)
                for rsu in self.rsu_list:
                    rsu.update_info()
                    rsu.run_step()
        
        except Exception as e:
            log_exception('run failed')

        finally:
            score, is_collision = self.oracle_manager.evaluate()

            try:
                if opt.record:
                    self.scenario_manager.client.stop_recorder()

                self.scenario_manager.close()

                for platoon in self.platoon_list:
                    platoon.destroy()
                for cav in self.single_cav_list:
                    # cav.perception_manager.rgb_camera[0].sensor: carla.Sensor
                    cav.destroy()
                for r in self.rsu_list:
                    r.destroy()
                for v in self.bg_veh_list:
                    v.destroy()
                del self.operation
                # self.scenario_manager.client.apply_batch(
                #     carla.command.DestroyActor(x) 
                #     for x in self.scenario_manager.world.get_actors()
                # )
                # settings = self.scenario_manager.world.get_settings()
                # settings.synchronous_mode = False
                # settings.fixed_delta_seconds = None
                # self.scenario_manager.world.apply_settings(settings)
            except Exception as e:
                logger.exception('destroy failed')

            finally:
                return score, is_collision


    def init_opt(self, scenario_params):
        ''' 初始化 opt 参数 '''
        opt.map = scenario_params.get('map')
        opt.sumo_dir = os.path.join(os.getcwd(), 'opencda', 'assets', opt.map)
        opt.xodr_dir = os.path.join(os.getcwd(), 'opencda', 'assets', opt.map)
        # platoon
        if scenario_params.get('scenario') \
            and scenario_params['scenario'].get('platoon_list') != []:
            opt.v2x = True
            opt.application = ['platooning']
        else:
            opt.v2x = False
            opt.application = ['single']
        
        # rsu
        if scenario_params.get('scenario') \
            and scenario_params['scenario'].get('rsu_list'):
            opt.rsu = True
        else:
            opt.rsu = False
        
        # traffic flow
        if scenario_params.get('sumo'):
            opt.sumo = True
            opt.sumo_cfg = opt.sumo_dir
            scenario_params['sumo']['gui'] = False
        else:
            opt.sumo = False
            opt.sumo_cfg = None
        
        # map
        if 'Town' not in opt.map:
            opt.xodr_file = os.path.join(opt.xodr_dir, opt.map + '.xodr')
            opt.map_helper = customized_map_api.spawn_helper_2lanefree
            opt.town = None
        else:
            opt.xodr_file = None
            opt.map_helper = None
            opt.town = opt.map
            
        opt.record_file = f"{map}_{'cosim' if opt.sumo else 'carla'}.log"
        

    # HACK: 应该在init之前变异字典
    def mutate(self, strategy=None):
        if strategy == 'weather':
            self.operation.set_weather()
            log_process_info('mutate weather')
        elif strategy == 'traffic' and not opt.sumo:
            self.operation.set_traffic()
            log_process_info('mutate traffic')
        elif strategy == 'actor' and not opt.sumo:
            self.operation.set_actor()
            log_process_info('mutate actor')
        else:
            self.mutate(random.choice(opt.mutate_world_strategy))

    @staticmethod
    def mutate_param(param, strategy=None):
        """
        Mutates the scenario parameters based on the given parameter dictionary.

        Args:
            param (dict): The parameter dictionary containing mutation settings.
        """
        if strategy == 'noise':
            operation.Operation.set_noise(param)
            log_process_info('mutate noise')
            return True
        elif strategy == 'platoon':
            operation.Operation.set_platoon(param)
            log_process_info('mutate platoon')
            return True
        else:
            strategy = random.choice(opt.all_strategy)
            if strategy in opt.mutate_param_strategy: 
                return Scenario.mutate_param(param, strategy)
            else:
                return False
            
            
    def init_actor(self, actor):
        """
        Initializes the actors in the simulation.

        Args:
            actor (carla.Actor): The actor's parameters.
        """
        self.mutator = {
            'vehicle_list': [],
            'walker_list': [],
            'rsu_list': []
        }
        vehicle_list = actor.get('vehicle_list', [])
        walker_list = actor.get('walker_list', [])
        rsu_list = actor.get('rsu_list', [])
        for vehicle in vehicle_list:
            self.spawn_vehicle(vehicle)
            self.mutator['vehicle_list'].append(vehicle)
        for walker in walker_list:
            self.spawn_walker(walker)
            self.mutator['walker_list'].append(walker)
        for rsu in rsu_list:
            self.spawn_rsu(rsu)
            self.mutator['rsu_list'].append(rsu)


    def spawn_vehicle(self, vehicle):
        """
        Spawns a vehicle in the simulation.

        Args:
            vehicle (carla.Vehicle): The vehicle's parameters.
        """
        world = self.scenario_manager.world
        blueprint = world.get_blueprint_library().find(vehicle.get('blueprint'))
        self.operation.spawn_vehicle(
            carla.Transform(
                carla.Location(x=vehicle.get('x'), y=vehicle.get('y'), z=vehicle.get('z')),
                carla.Rotation(pitch=vehicle.get('pitch'), yaw=vehicle.get('yaw'), roll=vehicle.get('roll'))
            ),
            blueprint,
            vehicle.get('strategy'),
            False
        )

        
    def spawn_walker(self, walker: dict):
        """
        Spawns a walker in the simulation.

        Args:
            walker (dict): The walker's parameters.
        """
        world = self.scenario_manager.world
        blueprint = world.get_blueprint_library().find(walker.get('blueprint'))
        transform = carla.Transform(
            carla.Location(x=walker.get('x'), y=walker.get('y'), z=walker.get('z')),
            carla.Rotation(pitch=walker.get('pitch'), yaw=walker.get('yaw'), roll=walker.get('roll'))
        )
        destination = walker.get('destination')
        self.operation.spawn_walker(
            opt.walker_speed_min,
            opt.walker_speed_max,
            walker.get('speed'),
            carla.Location(destination.x, destination.y, destination.z) if any(destination.values()) else None,
            transform,
            blueprint,
            False,
            walker.get('strategy')
        )

    
    def init_mutator(self):
        ''' 初始化变异器 '''
        mutator = self.raw_param.get('mutator',{})
        self.init_actor(mutator.get('actor', {}))

    
    def get_raw_param(self):
        """Override __dict__ to return the raw parameters."""
        # self.raw_param['sync_mode'] = True
        return self.raw_param


def make_and_run(scenario_params):
    seed = random.randint(1,9999)
    random.seed(seed)
    if not scenario_params.get('world'): scenario_params['world'] = {}
    # 变异opencda中的变量
    if not opt.raw: 
        scenario_params['world']['seed'] = seed
        is_mutated = Scenario.mutate_param(scenario_params)

    scenario = Scenario(scenario_params)

    # 变异carla中的变量 确保两者取其一变异
    if not opt.raw and not is_mutated: 
        scenario.mutate()

    score, is_collision = scenario.run()
    return (score, is_collision, copy.deepcopy(scenario.get_raw_param()))


def _wrapped_make_and_run(scenario_params, queue):
    """处理场景运行的包装函数，供多进程调用"""
    try:
        result = make_and_run(scenario_params)
        queue.put(result)
    except Exception as e:
        import traceback
        error_msg = f"Error occurred: {str(e)}\n{traceback.format_exc()}"
        queue.put(error_msg)

def process_run(scenario_params):
    # 创建队列用于获取结果
    ctx = mp.get_context('spawn')
    result_queue = ctx.Queue()
    p = ctx.Process(target=_wrapped_make_and_run, args=(scenario_params, result_queue))
    p.start()
    p.join(3 * 60)

    if p.is_alive():
        log_process_critical('time out, kill the process')
        p.kill()
    if not result_queue.empty():
        return result_queue.get()
    else: return None
    
    
# for test
if __name__ == '__main__':
    utils_.restart_carla()
    file = '/home/test/V2X/OpenCDA/OpenCDA/src/collision/platoon_joining_town06_carla_02.yaml'
    param = utils_.get_param(file)
    param['map'] = utils_.get_map_name(file)

    log_process_info(make_and_run(param))
    print(1)
    # utils_.restart_carla()
    
    # score, is_success, params = make_and_run(param)
    # print(score, is_success, params)
