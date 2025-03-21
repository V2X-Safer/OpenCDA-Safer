import os
import random
import carla
from opencda.core.application.platooning import platooning_manager
from opencda.core.common.cav_world import CavWorld
from opencda.core.common.vehicle_manager import VehicleManager
from opencda.scenario_testing.evaluations.evaluate_manager import EvaluationManager
from opencda.scenario_testing.utils import cosim_api, customized_map_api, sim_api
from typing import Union
from opencda.scenario_testing.utils.yaml_utils import add_current_time

from src import operation
from src.oracle_manager import OracleManager
import src.utils_ as utils_
import opt

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

    # HACK: 此处先初始化了, 应该先变异参数后在run时初始化
    def __init__(self, scenario_params):
        self.init_opt(scenario_params)
        utils_.debug_dict(opt.__dict__, opt.debug)
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

        if opt.v2x:
            self.scenario_manager_cls = cosim_api.CoScenarioManager
        else: 
            # ScenarioManager will not create a CavWorld
            manager_args['cav_world'] = CavWorld(opt.apply_ml)
            self.scenario_manager_cls = sim_api.ScenarioManager
        
        if opt.sumo_cfg:
            manager_args["sumo_file_parent_path"] = opt.sumo_cfg
        utils_.debug_dict(manager_args, opt.debug, ['scenario_params'])
        return self.scenario_manager_cls(**manager_args)


        
    def run(self):
        ''' 运行场景 '''
        try: 
            while True:
                # HACK: 应该OpenCDA的一个bug 需要手动tick一次cav_world
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

        finally:
            score, is_success = self.oracle_manager.evaluate()

            try:
                if opt.record:
                    self.scenario_manager.client.stop_recorder()

                self.scenario_manager.close()

                # utils_.check_carla()
                for platoon in self.platoon_list:
                    platoon.destroy()
                for cav in self.single_cav_list:
                    # cav.perception_manager.rgb_camera[0].sensor: carla.Sensor
                    cav.destroy()
                for r in self.rsu_list:
                    r.destroy()
                for v in self.bg_veh_list:
                    v.destroy()
            finally:
                # utils_.check_carla()
                return score, is_success


    def init_opt(self, scenario_params):
        ''' 初始化 opt 参数 '''
        opt.map = scenario_params.get('map')
        # platoon 
        if scenario_params.get('scenario') \
            and scenario_params['scenario'].get('platoon_list'):
            opt.v2x = True
            opt.application = ['platooning']
        
        # rsu
        if scenario_params.get('scenario') \
            and scenario_params['scenario'].get('rsu_list'):
            opt.rsu = True
        
        # traffic flow
        if scenario_params.get('sumo'):
            opt.sumo = True
            opt.sumo_cfg = os.path.join(opt.sumo_dir, opt.map)
        
        # map
        if 'Town' not in opt.map:
            opt.xodr_file = os.path.join(opt.xodr_dir, opt.map + '.xodr')
            opt.map_helper = customized_map_api.spawn_helper_2lanefree
            opt.town = None
            
        

    
    def set_traffic(self):
        ''' 设置交通流 '''
        pass


    def set_task(self):
        ''' 设置任务 '''
        pass

    def mutate(self):
        ''' 变异场景 '''
        # TODO: 改变场景参数
        if opt.mutate_strategy == 'weather':
            self.operation.set_weather()
        elif opt.mutate_strategy == 'traffic':
            self.operation.set_traffic()
        elif opt.mutate_strategy == 'task':
            self.operation.set_task()
        else:
            random.choice([self.operation.set_weather, self.operation.set_traffic, self.operation.set_task])()

    def __dict__(self):
        """Override __dict__ to return the raw parameters."""
        return self.raw_param
        


if __name__ == '__main__':
    # for test
    seed_param = utils_.get_param('test.yaml')
    Scenario = Scenario(seed_param)
    Scenario.run()