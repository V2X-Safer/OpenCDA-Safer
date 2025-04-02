import stat
import carla
from random import randrange
import random

import carla.libcarla
from opencda.core.common.cav_world import CavWorld
from opencda.scenario_testing.utils.sim_api import ScenarioManager
from opencda.scenario_testing.evaluations.evaluate_manager import EvaluationManager
from opencda.scenario_testing.utils.cosim_api import CoScenarioManager
from opencda.scenario_testing.utils.yaml_utils import add_current_time
from opencda.core.application.platooning.platooning_manager import PlatooningManager

import opt

class Operation:
    ''' 封装一层 carla 和 opencda 的接口'''
    def __init__(self, scenario):
        from src.Scenario_ import Scenario

        self.scenario: Scenario = scenario
        self.cav_world: CavWorld= self.scenario.scenario_manager.cav_world
        self.scenario_manager: ScenarioManager = self.scenario.scenario_manager
        self.actor_list: list[carla.Actor] = []
    

    @property
    def spawn_point(self):
        return random.choice(self.cav_world.get_map().get_spawn_points())
    
    @property
    def blueprints(self):
        return self.scenario_manager.world.get_blueprint_library()

    @property
    def all_points(self):
        return self.scenario_manager.carla_map.get_spawn_points()

    @property
    def route(self):
        ''' 生成两个随机点,检测距离是否符合要求 '''
        start = self.spawn_point
        end = self.spawn_point
        # 检测两点距离
        while opt.spawn_min_distance > start.location.distance(end.location) \
            or start.location.distance(end.location) > opt.spawn_max_distance:
            end = self.spawn_point
        return start, end 

    @property
    def weather(self):
        return random.choice([self.dark_weather, self.light_weather])

    @property
    def dark_weather(self):
        ''' 强 雨 雾 风 水坑 晚上 '''
        return carla.WeatherParameters(
            cloudiness=random.uniform(opt.cloudiness_min, opt.cloudiness_max),
            precipitation=random.uniform(opt.precipitation_min, opt.precipitation_max),
            precipitation_deposits=random.uniform(opt.precipitation_deposits_min, opt.precipitation_deposits_max),
            wind_intensity=random.uniform(opt.wind_intensity_min, opt.wind_intensity_max),
            fog_density=random.uniform(opt.fog_density_min, opt.fog_density_max),
            fog_distance=random.uniform(opt.fog_distance_min, opt.fog_distance_max),
            fog_falloff=random.uniform(0, opt.maxsize),  # 0 to inf
            wetness=random.uniform(opt.wetness_min, opt.wetness_max),
            sun_altitude_angle=random.uniform(opt.sun_altitude_angle_min, 0),  # 负值表示夜间
            sun_azimuth_angle=random.uniform(opt.sun_azimuth_angle_min, opt.sun_azimuth_angle_max)
        )

    @property
    def light_weather(self):
        ''' 弱 雨 雾 风 水坑 早上 '''
        return carla.WeatherParameters(
            cloudiness=random.uniform(0, opt.cloudiness_max/2),
            precipitation=random.uniform(0, opt.precipitation_max/5),
            precipitation_deposits=random.uniform(0, opt.precipitation_deposits_max/5),
            wind_intensity=random.uniform(0, opt.wind_intensity_max/2),
            fog_density=random.uniform(0, opt.fog_density_max/2),
            fog_distance=random.uniform(opt.fog_distance_min, opt.fog_distance_max/2),
            fog_falloff=random.uniform(0, opt.maxsize/2),  # 0 to inf
            wetness=random.uniform(0, opt.wetness_max/5),
            sun_altitude_angle=random.uniform(0, opt.sun_altitude_angle_max),  # 正值表示白天
            sun_azimuth_angle=random.uniform(opt.sun_azimuth_angle_min, opt.sun_azimuth_angle_max)
        )

    
    def get_traffic(self):
        return self.scenario.traffic_manager

    def get_spawn_point(self, distance: int = opt.near_distance):
        spawn_point = random.choice(self.all_points)
        # 确保生成位置在参数范围内，并且不小于最小距离
        while not (spawn_point.location.distance(self.scenario.spectator_vehicle.get_transform().location) <= distance \
            and spawn_point.location.distance(self.scenario.spectator_vehicle.get_transform().location) > opt.vehicle_min_distance \
            and spawn_point is not None):
            spawn_point = random.choice(self.all_points)
        return spawn_point
    

    def set_weather(self, weather = None):
        """
        设置天气并更新scenario.raw_param
        
        Args:
            weather: 天气参数,如果为None则随机选择
        """
        if not weather:
            weather = self.weather
        self.scenario_manager.world.set_weather(weather)
        
        # 更新raw_param中的天气参数
        if 'world' not in self.scenario.raw_param:
            self.scenario.raw_param['world'] = {}
        
        # 提取weather参数并更新到raw_param
        self.scenario.raw_param['world']['weather'] = {
            'sun_altitude_angle': weather.sun_altitude_angle,
            'cloudiness': weather.cloudiness,
            'precipitation': weather.precipitation,
            'precipitation_deposits': weather.precipitation_deposits,
            'wind_intensity': weather.wind_intensity,
            'fog_density': weather.fog_density,
            'fog_distance': weather.fog_distance,
            'fog_falloff': weather.fog_falloff,
            'wetness': weather.wetness,
            'sun_azimuth_angel': weather.sun_azimuth_angle
        }


    def set_traffic(self, density = None):
        """
        设置交通流密度并更新scenario.raw_param
        """
        # 获取交通管理器
        traffic_manager = self.get_traffic()
        
        # 如果没有指定密度,随机生成
        if density is None:
            density = random.randint(5, 30)
        
        # 设置交通流的全局距离（越小车辆越密集）
        global_distance = max(1, 30 - density)
        traffic_manager.set_global_distance_to_leading_vehicle(global_distance)
        
        # 更新raw_param
        if 'carla_traffic_manager' not in self.scenario.raw_param:
            self.scenario.raw_param['carla_traffic_manager'] = {}
        
        self.scenario.raw_param['carla_traffic_manager']['global_distance'] = global_distance


    def set_actor(self, spawn_point: carla.Transform = None, vehicle_strategy: str = 'random', walker_strategy: str = 'random'):
        ''' 设置actor '''
        if not spawn_point: spawn_point = self.get_spawn_point()
        return self.spawn_vehicle(spawn_point=spawn_point, strategy=vehicle_strategy)
        if random.choice([True, False]):
            return self.spawn_vehicle(spawn_point=spawn_point, strategy=vehicle_strategy)
        else:
            return self.spawn_walker(spawn_point=spawn_point, strategy=walker_strategy)


    def spawn_vehicle(self,
                     spawn_point: carla.Transform = None, 
                     vehicle_bp: carla.ActorBlueprint = None,
                     strategy: str = 'random',
                     is_append: bool = True):
        """
        在指定位置生成车辆,并将信息保存到raw_param
        
        Args:
            spawn_point: 生成位置,如果为None则在观察者车辆附近生成
            vehicle_bp: 车辆蓝图,如果为None则随机选择
            strategy: 控制策略：'ai'使用CARLA自动驾驶,'linear'使用线性移动,'random'随机选择
            is_append: 是否将车辆信息添加到raw_param
            
        Returns:
            生成的车辆对象
        """
        # 如果未指定生成点,则在观察者车辆附近选择
        if spawn_point is None:
            spawn_point = self.get_spawn_point()
        
        if not vehicle_bp:
            vehicle_bp = random.choice(self.blueprints.filter('vehicle.*'))
        
        # 尝试生成车辆
        vehicle = self.scenario_manager.world.try_spawn_actor(vehicle_bp, spawn_point)

        
        while not vehicle:
            spawn_point = self.get_spawn_point()
            vehicle = self.scenario_manager.world.try_spawn_actor(vehicle_bp, spawn_point)
        self.actor_list.append(vehicle)
        
        # 根据策略设置车辆行为
        if strategy == 'ai':
            # 使用CARLA的交通管理器自动驾驶
            traffic_manager = self.get_traffic()
            vehicle.set_autopilot(True, traffic_manager.get_port())
            
            # 随机设置车速差异
            speed_diff = random.randint(-30, 30)
            traffic_manager.vehicle_percentage_speed_difference(vehicle, speed_diff)
            
            controller_info = {
                'type': 'autopilot',
                'speed_diff': speed_diff,
                'tm_port': traffic_manager.get_port()
            }
        elif strategy == 'linear':
            # 线性移动控制
            control = carla.VehicleControl()
            control.throttle = random.uniform(opt.throttle_min, opt.throttle_max)
            control.steer = random.uniform(opt.steer_min, opt.steer_max)
            control.brake = random.uniform(opt.brake_min, opt.brake_max)
            # 随机添加手刹和倒车状态
            control.hand_brake = random.random() < opt.hand_brake_prob
            control.reverse = random.random() < opt.reverse_prob
            vehicle.apply_control(control)
            
            controller_info = {
                'type': 'linear',
                'throttle': control.throttle,
                'steer': control.steer,
                'brake': control.brake,
                'hand_brake': control.hand_brake,
                'reverse': control.reverse
            }
        else:  # random或其他情况
            # 随机选择移动方式
            if random.choice([True, False]):
                traffic_manager = self.get_traffic()
                vehicle.set_autopilot(True, traffic_manager.get_port())
                speed_diff = random.randint(-30, 30)
                traffic_manager.vehicle_percentage_speed_difference(vehicle, speed_diff)
                
                controller_info = {
                    'type': 'autopilot',
                    'speed_diff': speed_diff,
                    'tm_port': traffic_manager.get_port()
                }
            else:
                control = carla.VehicleControl()
                control.throttle = random.uniform(opt.throttle_min, opt.throttle_max)
                control.steer = random.uniform(opt.steer_min, opt.steer_max)
                control.brake = random.uniform(opt.brake_min, opt.brake_max)
                # 随机添加手刹和倒车状态
                control.hand_brake = random.random() < opt.hand_brake_prob
                control.reverse = random.random() < opt.reverse_prob
                vehicle.apply_control(control)
                
                controller_info = {
                    'type': 'linear',
                    'throttle': control.throttle,
                    'steer': control.steer,
                    'brake': control.brake,
                    'hand_brake': control.hand_brake,
                    'reverse': control.reverse
                }
        
        if is_append:
            # 确保mutator字段存在
            if 'mutator' not in self.scenario.raw_param:
                self.scenario.raw_param['mutator'] = {}
            
            # 确保actor字段存在
            if 'actor' not in self.scenario.raw_param['mutator']:
                self.scenario.raw_param['mutator']['actor'] = {}
                
            # 确保vehicle字段是列表
            if 'vehicle_list' not in self.scenario.raw_param['mutator']['actor']:
                self.scenario.raw_param['mutator']['actor']['vehicle_list'] = []
            
            # 添加车辆信息到raw_param
            vehicle_info = {
                'blueprint': vehicle_bp.id,
                'x': spawn_point.location.x,
                'y': spawn_point.location.y, 
                'z': spawn_point.location.z,
                'pitch': spawn_point.rotation.pitch,
                'yaw': spawn_point.rotation.yaw,
                'roll': spawn_point.rotation.roll,
                'controller': controller_info,
                'strategy': strategy,
            }
            
            # 添加额外的车辆属性
            # for attr in vehicle_bp.get_attribute_names():
            #     vehicle_info['attributes'][attr] = vehicle_bp.get_attribute(attr).as_str()
            
            # 获取车辆物理特性
            # physics_control = vehicle.get_physics_control()
            # vehicle_info['physics'] = {
            #     'mass': physics_control.mass,
            #     'max_rpm': physics_control.max_rpm,
            #     'moi': physics_control.moi,
            #     'drag_coefficient': physics_control.drag_coefficient
            # }
            
            self.scenario.raw_param['mutator']['actor']['vehicle_list'].append(vehicle_info)
        
        return vehicle

    def spawn_walker(self,
                     min_speed: float = None,
                     max_speed: float = None,
                     current_speed: float = None,
                     destination: carla.Location = None,
                     spawn_point: carla.Transform = None,
                     walker_bp: carla.libcarla.ActorBlueprint = None,
                     is_append: bool = True,
                     strategy: str = 'random'):

        if spawn_point is None:
            spawn_point = self.get_spawn_point()
        if not walker_bp: 
            walker_bp = random.choice(self.blueprints.filter('walker.*'))
        
        walker = self.scenario_manager.world.try_spawn_actor(walker_bp, spawn_point)
        
        while not walker:
            spawn_point = self.get_spawn_point()
            walker = self.scenario_manager.world.try_spawn_actor(walker_bp, spawn_point)
        self.actor_list.append(walker)
            
        walker_controller_bp = None
        walker_controller = None

        if strategy == 'ai':
            walker_controller_bp = self.blueprints.find('controller.ai.walker')
        else:
            walker_controller_bp = random.choice([self.blueprints.find('controller.ai.walker'), None])

        # AI control
        if walker_controller_bp:
            walker_controller = self.scenario_manager.world.spawn_actor(walker_controller_bp, spawn_point, walker)
            self.actor_list.append(walker_controller)
            # world.tick()
            walker_controller.start()
            current_speed = random.uniform(
                opt.walker_speed_min if not min_speed else min_speed,
                opt.walker_speed_max if not max_speed else max_speed
            ) if not current_speed else current_speed
            
            # Set walker AI controller to follow the walker
            walker_controller.go_to_location(
                self.scenario_manager.world.get_random_location_from_navigation()
                if not destination else destination
            )
            walker_controller.set_max_speed(current_speed)
                
        # linear go straight to destination
        else:
            forward_vec = spawn_point.rotation.get_forward_vector()
            walker_controller = carla.WalkerControl()
            walker_controller.direction = forward_vec
            current_speed = random.uniform(opt.walker_speed_min, opt.walker_speed_max)
            walker_controller.speed = current_speed
            walker.apply_control(walker_controller)
                
        if is_append:
            # 确保mutator字段存在
            if 'mutator' not in self.scenario.raw_param:
                self.scenario.raw_param['mutator'] = {}

            # 确保actor字段存在
            if 'actor' not in self.scenario.raw_param['mutator']:
                self.scenario.raw_param['mutator']['actor'] = {}
                
            # 确保walker_list字段是列表
            if 'walker_list' not in self.scenario.raw_param['mutator']['actor']:
                self.scenario.raw_param['mutator']['actor']['walker_list'] = []

            # 添加行人信息到raw_param
            walker_info = {
                'blueprint': walker_bp.id,
                'x': spawn_point.location.x,
                'y': spawn_point.location.y, 
                'z': spawn_point.location.z,
                'pitch': spawn_point.rotation.pitch,
                'yaw': spawn_point.rotation.yaw,
                'roll': spawn_point.rotation.roll,
                'strategy': strategy,
                'destination': {
                    'x': destination.x if destination else None,
                    'y': destination.y if destination else None,
                    'z': destination.z if destination else None
                },
                'speed': current_speed
            }
            
            # 添加到正确的路径
            self.scenario.raw_param['mutator']['actor']['walker_list'].append(walker_info)
            
        return walker
    
    
    @staticmethod
    def set_noise(param):
        """
        Add random sensing noise parameters to RSU and vehicle sensors.
        
        This function adds random noise values to various sensing components:
        - RSU lidar perception and localization sensors
        - Vehicle lidar perception and localization sensors
        - Single CAV location, yaw, speed noise and communication lag
        
        Args:
            param (dict): Configuration dictionary containing sensor parameters
        """
        # add rsu sensing noise
        rsu_sensing = param.get('rsu_base',{}).get('sensing',{})
        rsu_sensing_lidar = rsu_sensing.get('perception',{}).get('lidar', {})
        rsu_sensing_location = rsu_sensing.get('locatization', {})
        single_cav_list = param.get('scenario', {}).get('single_cav_list', {})
        if rsu_sensing_lidar:
            rsu_sensing_lidar['noise_stddev'] = random.uniform(opt.lidar_noise_min, opt.lidar_noise_max)
        if rsu_sensing_location:
            rsu_sensing_location['activate'] = True
            rsu_sensing_location['noise_alt_stddev'] = random.uniform(opt.alt_noise_min, opt.alt_noise_max)
            rsu_sensing_location['noise_lat_stddev'] = random.uniform(opt.lat_noise_min, opt.lat_noise_max)
            rsu_sensing_location['noise_lon_stddev'] = random.uniform(opt.lon_noise_min, opt.lon_noise_max)
        
        # add vehicle sensing noise
        vehicle_sensing = param.get('vehicle_base',{}).get('sensing',{})
        vehicle_sensing_lidar = vehicle_sensing.get('perception',{}).get('lidar', {})
        vehicle_sensing_location = vehicle_sensing.get('locatization', {})
        if vehicle_sensing_lidar:
            vehicle_sensing_lidar['noise_stddev'] = random.uniform(opt.lidar_noise_min, opt.lidar_noise_max)
        if vehicle_sensing_location:
            vehicle_sensing_location['activate'] = True
            vehicle_sensing_location['noise_alt_stddev'] = random.uniform(opt.alt_noise_min, opt.alt_noise_max)
            vehicle_sensing_location['noise_lat_stddev'] = random.uniform(opt.lat_noise_min, opt.lat_noise_max)
            vehicle_sensing_location['noise_lon_stddev'] = random.uniform(opt.lon_noise_min, opt.lon_noise_max)
        if single_cav_list:
            for single_cav in single_cav_list:
                single_cav: dict
                single_cav['loc_noise'] = random.uniform(opt.loc_noise_min, opt.loc_noise_max)
                single_cav['yaw_noise'] = random.uniform(opt.yaw_noise_min, opt.yaw_noise_max)
                single_cav['speed_noise'] = random.uniform(opt.speed_noise_min, opt.speed_noise_max)
                single_cav['lag'] = random.uniform(opt.lag_min, opt.lag_max)
    
    @staticmethod
    def set_platoon(param: dict):
        """
        Configure platoon parameters with random values within defined ranges.
        
        This function sets various platoon and V2X communication parameters randomly:
        - For platoons: inter_gap, open_gap, and warm_up_speed
        - For single CAVs: communication_range for V2X systems
        
        Args:
            param (dict): Configuration dictionary containing scenario parameters
        """
        platoon_list = param.get('scenario', {}).get('platoon_list', {})
        single_cav_list = param.get('scenario', {}).get('single_cav_list', {})
        if platoon_list:
            for platoon in platoon_list:
                platoon:dict
                platoon['inter_gap'] = random.uniform(opt.inter_gap_min, opt.inter_gap_max)
                platoon['open_gap'] = random.uniform(opt.open_gap_min, opt.open_gap_max)
                platoon['warm_up_speed'] = random.uniform(opt.warm_up_speed_min, opt.warm_up_speed_max)
        
        if single_cav_list:
            for single_cav in single_cav_list:
                single_cav:dict
                v2x = single_cav.get('v2x')
                if v2x:
                    v2x['communication_range'] = random.uniform(opt.communication_range_min, opt.communication_range_max)



    def __del__(self):
        for actor in self.actor_list:
            if actor.is_alive:
                actor.destroy()


