
from state.state import State

class ScenarioState(State):
    def __init__(self, scenario_params, manager_list):
        self.weather = {
                    'sun_altitude_angle': scenario_params.world.sun_altitude_ang,  # 90 is the midday and -90 is the midnight
                    'cloudiness': scenario_params.world.cloudiness,  # 0 is the clean sky and 100 is the thickest cloud
                    'precipitation': scenario_params.world.precipitation,  # rain, 100 is the heaviest rain
                    'precipitation_deposits': scenario_params.world.precipitation_deposits,  # Determines the creation of puddles. Values range from 0 to 100, being 0 none at all and 100 a road completely capped with water.
                    'wind_intensity': scenario_params.world.wind_intensity,  # it will influence the rain
                    'fog_density': scenario_params.world.fog_density,  # fog thickness, 100 is the largest
                    'fog_distance': scenario_params.world.fog_distance,  # Fog start distance. Values range from 0 to infinite.
                    'fog_falloff': scenario_params.world.fog_falloff,  # Density of the fog (as in specific mass) from 0 to infinity. The bigger the value, the more dense and heavy it will be, and the fog will reach smaller heights
                    'wetness': scenario_params.world.wetness
                }
        if scenario_params.carla_traffic_manager:
            self.carla_traffic_manager = {
                'sync_mode': scenario_params.carla_traffic_manager.sync_mode,
                'global_distance': scenario_params.carla_traffic_manager.global_distance,
                'global_speed_perc': scenario_params.carla_traffic_manager.global_speed_perc,
                'set_osm_mode': scenario_params.carla_traffic_manager.set_osm_mode,
                'auto_lane_change': scenario_params.carla_traffic_manager.auto_lane_change,
                'ignore_lights_percentage': scenario_params.carla_traffic_manager.ignore_lights_percentage,
                'random': scenario_params.carla_traffic_manager.random,
                'vehicle_list': scenario_params.carla_traffic_manager.vehicle_list,
                'range': scenario_params.carla_traffic_manager.range
            }
        elif scenario_params.sumo:
            self.sumo = {
                'port': scenario_params.sumo.port,
                'host': scenario_params.sumo.host,
                'gui': scenario_params.sumo.gui,
                'client_order': scenario_params.sumo.client_order,
                'step_length': scenario_params.sumo.step_length
            }


        # TODO: 路基单元 有待考究 
        self.rsu_base = {
            
        }


