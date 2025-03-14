import opt
import random
import carla
from random import randrange
from opencda.scenario_testing.utils.sim_api import ScenarioManager
from opencda.scenario_testing.evaluations.evaluate_manager import EvaluationManager
from opencda.scenario_testing.utils.cosim_api import CoSimManager
from opencda.scenario_testing.utils.yaml_utils import add_current_time
from opencda.core.application.platooning.platooning_manager import PlatooningManager


class Operator:
    ''' 封装一层 carla 和 opencda 的接口'''
    def __init__(self, platoon_manager_list, scenario_manager, cav_world):
        self.cav_world: carla.World = cav_world
        self.scenario_manager: ScenarioManager = scenario_manager
        self.platoon_manager_list: list[PlatooningManager] = platoon_manager_list


    
    def set_weather(self):
        weather = carla.WeatherParameters(self.weather)
        self.cav_world.set_weather(weather)

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
            fog_density=randrange(0,float.max),  # 0 to inf
            fog_distance=randrange(0,100),
            fog_falloff=randrange(0,float.max), # 0 to inf
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
            fog_density=randrange(0,float.max),  # 0 to inf
            fog_distance=randrange(0,50),
            fog_falloff=randrange(0,float.max), # 0 to inf
            wetness=randrange(0,20),
            sun_altitude_angle=randrange(0,180),
            sun_azimuth_angle=randrange(0,90)
        )


    @property
    def spawn_point(self):
        return random.choice(self.cav_world.get_map().get_spawn_points())

    @property
    def route(self):
        ''' 生成两个随机点，检测距离是否符合要求 '''
        start = self.spawn_point
        end = self.spawn_point
        # 检测两点距离
        while opt.min_distance > start.location.distance(end.location) \
            or start.location.distance(end.location) > opt.max_distance:
            end = self.spawn_point
        return start, end 
    
    
    def get_traffic(self):
        return self.scenario_manager.get_trafficmanager()

    def get_vehicles(self, index=0):
        return self.platoon_manager_list[index]


