import carla
from random import randrange
import random
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


    
    def set_weather(self, weather = None):
        """
        设置天气并更新scenario.raw_param
        
        Args:
            weather: 天气参数，如果为None则随机选择
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


    @property
    def weather(self):
        return random.choice([self.dark_weather, self.light_weather])

    @property
    def dark_weather(self):
        ''' 强 雨 雾 风 水坑 晚上 '''
        return carla.WeatherParameters(
            cloudiness=randrange(0,100),
            precipitation=randrange(0,100),
            precipitation_deposits=randrange(0,100),
            wind_intensity=randrange(0,100),
            fog_density=randrange(0,opt.maxsize), # 0 to inf
            fog_distance=randrange(0,100),
            fog_falloff=randrange(0,opt.maxsize), # 0 to inf
            wetness=randrange(0,100),
            sun_altitude_angle=randrange(0,180),
            sun_azimuth_angle=randrange(-90,0)
        )

    @property
    def light_weather(self):
        ''' 弱 雨 雾 风 水坑 早上 '''
        return carla.WeatherParameters(
            cloudiness=randrange(0,50),
            precipitation=randrange(0,20),
            precipitation_deposits=randrange(0,20),
            wind_intensity=randrange(0,50),
            fog_density=randrange(0,opt.maxsize),  # 0 to inf
            fog_distance=randrange(0,50),
            fog_falloff=randrange(0,opt.maxsize), # 0 to inf
            wetness=randrange(0,20),
            sun_altitude_angle=randrange(0,180),
            sun_azimuth_angle=randrange(0,90)
        )

    def set_waypoints(self, start = None, end = None):
        """
        设置路线并更新scenario.raw_param
        
        Args:
            start: 起始点，如果为None则随机生成
            end: 终点，如果为None则随机生成
        """
        if not start or not end:
            start, end = self.route
        self.scenario_manager.set_route(start, end)
        
        # 更新raw_param中的路线参数
        if 'scenario' not in self.scenario.raw_param:
            self.scenario.raw_param['scenario'] = {}
        
        # 添加或更新路线信息
        route_info = {
            'start_point': {
                'x': start.location.x,
                'y': start.location.y,
                'z': start.location.z,
                'yaw': start.rotation.yaw
            },
            'end_point': {
                'x': end.location.x,
                'y': end.location.y,
                'z': end.location.z,
                'yaw': end.rotation.yaw
            }
        }
        
        # 更新到raw_param
        self.scenario.raw_param['scenario']['route'] = route_info
    
    @property
    def spawn_point(self):
        return random.choice(self.cav_world.get_map().get_spawn_points())

    @property
    def route(self):
        ''' 生成两个随机点，检测距离是否符合要求 '''
        start = self.spawn_point
        end = self.spawn_point
        # 检测两点距离
        while opt.spawn_min_distance > start.location.distance(end.location) \
            or start.location.distance(end.location) > opt.spawn_max_distance:
            end = self.spawn_point
        return start, end 
    

    def platoon_spawn_point(self, num = 1):
        ''' 生成 num 个连续随机点 '''
        all_points = self.cav_world.get_map().get_spawn_points()
        random_start = random.randint(0, len(all_points) - 1)
        # 构造一个循环
        all_points *= 2
        spawn_points = []
        for index, points in enumerate(all_points[random_start:]):
            start = index
            begin_loc = points.location
            while start < num and start < len(all_points):
                if begin_loc.distance(all_points[start].location) <= opt.platoon_distance :
                    spawn_points.append(all_points[start])
                start += 1
            if len(spawn_points) == num:
                break
            else: spawn_points.clear()
        
        if spawn_points:
            # 更新raw_param中的生成点
            if 'scenario' not in self.scenario.raw_param:
                self.scenario.raw_param['scenario'] = {}
            
            # 准备存储生成点信息
            spawn_info = []
            for i, point in enumerate(spawn_points):
                spawn_info.append({
                    'x': point.location.x,
                    'y': point.location.y,
                    'z': point.location.z,
                    'yaw': point.rotation.yaw
                })
            
            # 更新platoon_spawn_points
            self.scenario.raw_param['scenario']['platoon_spawn_points'] = spawn_info
            
            return spawn_points
        else:
            raise Exception('No enough spawn points')
            
    
    def set_traffic(self):
        """
        设置交通流密度并更新scenario.raw_param
        """
        # 获取交通管理器
        traffic_manager = self.get_traffic()
        
        # 如果没有指定密度，随机生成
        if density is None:
            density = random.randint(5, 30)
        
        # 设置交通流的全局距离（越小车辆越密集）
        global_distance = max(1, 30 - density)
        traffic_manager.set_global_distance_to_leading_vehicle(global_distance)
        
        # 更新raw_param
        if 'carla_traffic_manager' not in self.scenario.raw_param:
            self.scenario.raw_param['carla_traffic_manager'] = {}
        
        self.scenario.raw_param['carla_traffic_manager']['global_distance'] = global_distance
    
    def set_task(self, task_type=None):
        pass
    

    def get_traffic(self):
        return self.scenario_manager.get_trafficmanager()


