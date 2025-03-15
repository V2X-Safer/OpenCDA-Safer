import carla
from opencda.core.application.platooning import platooning_manager
from opencda.core.common.vehicle_manager import VehicleManager
from opencda.scenario_testing.evaluations.evaluate_manager import EvaluationManager
from opencda.scenario_testing.utils import cosim_api, sim_api
from typing import Union


import opt

class scenario:
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
        
    eval_manager : EvaluationManager
        评估管理器实例，负责场景测试过程中的数据收集和性能评估。
        
    spectator : carla.Actor
        观察者视角实例，用于控制场景渲染的摄像机视角，默认设置为俯视视角。
    '''

    # HACK: 不够优雅
    def __init__(self, scenario_params):
        self.init_opt(scenario_params)
        self.scenario_manager = self.ScenarioManager(scenario_params)

        if opt.v2x:
            self.platoon_list = self.scenario_manager.create_platoon_manager(
                # TODO: add opt
                map_helper=opt.map_helper,
                data_dump=opt.data_dump)

        self.single_cav_list: list[VehicleManager] =  self.scenario_manager.create_vehicle_manager(
            # TODO: add opt
            application=opt.application,
            map_helper=opt.map_helper,
            data_dump=opt.data_dump
        )
        
        if opt.record:
            self.scenario_manager.client.start_recorder(
                opt.record_file,
                # TODO: add opt
                opt.addtional_recorder)

        if opt.carla:
            self.traffic_manager, self.bg_veh_list  = self.scenario_manager.create_traffic_carla()

        self.evaluation_manager = EvaluationManager(self.scenario_manager.cav_world,
                                              script_name=opt.record_file,
                                              current_time=scenario_params['current_time'])
        self.spectator = self.scenario_manager.world.get_spectator()
        if opt.v2x:
            spectator_vehicle = self.platoon_list[0].vehicle_manager_list[1].vehicle
        
    def ScenarioManager(self, scenario_params) -> Union[sim_api.ScenarioManager, sim_api.CoScenarioManager]:
        """
        Creates and returns the appropriate ScenarioManager based on configuration.
        
        Returns:
            Union[sim_api.ScenarioManager, cosim_api.CoScenarioManager]: 
                CoScenarioManager if SUMO is enabled, otherwise ScenarioManager
        """
        if opt.sumo:
            return cosim_api.CoScenarioManager(scenario_params,
                                             opt.apply_ml,
                                             opt.version,
                                             town=opt.town,
                                             cav_world=self.scenario_manager.cav_world,
                                             sumo_file_parent_path=opt.sumo_cfg)
        else: 
            return sim_api.ScenarioManager(scenario_params,
                                           opt.apply_ml,
                                           opt.version,
                                           town=opt.town,
                                           cav_world=self.scenario_manager.cav_world)
        
    def run(self):
        ''' 运行场景 '''
        try: 
            while True:
                self.scenario_manager.tick()
                transform = self.spectator_vehicle.get_transform()
                self.spectator.set_transform(
                    carla.Transform(
                        transform.location +
                    carla.Location(
                            z=80),
                    carla.Rotation(
                            pitch=-
                            90)))
                for platoon in self.platoon_list:
                    platoon.update_information()

                    platoon.run_step()
        except Exception as e:

            self.eval_manager.evaluate()

            if opt.record:

                self.scenario_manager.client.stop_recorder()

            self.scenario_manager.close()

            for platoon in self.platoon_list:
                platoon.destroy()
            for cav in self.single_cav_list:

                cav.destroy()
            for v in self.bg_veh_list:
                v.destroy()

    def init_opt(self, scenario_params):
        ''' 初始化 opt 参数 '''
        if scenario_params.get('scenario') \
            and scenario_params['scenario'].get('platoon_list'):
            opt.v2x = True
        else: 
            opt.v2x = False
