from state.state import State

class VehicleState(State):
    def __init__(self, opt):
        self.sensing = {
            'perception': {
                'activate': opt.vehicle_base.sensing.perception.activate,
                'camera': {
                    'visualize': opt.vehicle_base.sensing.perception.camera.visualize,
                    'num': opt.vehicle_base.sensing.perception.camera.num,
                    'positions': opt.vehicle_base.sensing.perception.camera.positions
                },
                'lidar': {
                    'visualize': opt.vehicle_base.sensing.perception.lidar.visualize,
                    'channels': opt.vehicle_base.sensing.perception.lidar.channels,
                    'range': opt.vehicle_base.sensing.perception.lidar.range,
                    'points_per_second': opt.vehicle_base.sensing.perception.lidar.points_per_second,
                    'rotation_frequency': opt.vehicle_base.sensing.perception.lidar.rotation_frequency,
                    'upper_fov': opt.vehicle_base.sensing.perception.lidar.upper_fov,
                    'lower_fov': opt.vehicle_base.sensing.perception.lidar.lower_fov,
                    'dropoff_general_rate': opt.vehicle_base.sensing.perception.lidar.dropoff_general_rate,
                    'dropoff_intensity_limit': opt.vehicle_base.sensing.perception.lidar.dropoff_intensity_limit,
                    'dropoff_zero_intensity': opt.vehicle_base.sensing.perception.lidar.dropoff_zero_intensity,
                    'noise_stddev': opt.vehicle_base.sensing.perception.lidar.noise_stddev
                }
            },
            'localization': {
                'activate': opt.vehicle_base.sensing.localization.activate,
                'dt': opt.vehicle_base.sensing.localization.dt,
                'gnss': {
                    'noise_alt_stddev': opt.vehicle_base.sensing.localization.gnss.noise_alt_stddev,
                    'noise_lat_stddev': opt.vehicle_base.sensing.localization.gnss.noise_lat_stddev,
                    'noise_lon_stddev': opt.vehicle_base.sensing.localization.gnss.noise_lon_stddev,
                    'heading_direction_stddev': opt.vehicle_base.sensing.localization.gnss.heading_direction_stddev,
                    'speed_stddev': opt.vehicle_base.sensing.localization.gnss.speed_stddev
                },
                'debug_helper': opt.vehicle_base.sensing.localization.debug_helper
            }
        }
        
        self.map_manager = {
            'pixels_per_meter': opt.vehicle_base.map_manager.pixels_per_meter,
            'raster_size': opt.vehicle_base.map_manager.raster_size,
            'lane_sample_resolution': opt.vehicle_base.map_manager.lane_sample_resolution,
            'visualize': opt.vehicle_base.map_manager.visualize,
            'activate': opt.vehicle_base.map_manager.activate
        }
        
        self.behavior = {
            'max_speed': opt.vehicle_base.behavior.max_speed,
            'tailgate_speed': opt.vehicle_base.behavior.tailgate_speed,
            'speed_lim_dist': opt.vehicle_base.behavior.speed_lim_dist,
            'speed_decrease': opt.vehicle_base.behavior.speed_decrease,
            'safety_time': opt.vehicle_base.behavior.safety_time,
            'emergency_param': opt.vehicle_base.behavior.emergency_param,
            'ignore_traffic_light': opt.vehicle_base.behavior.ignore_traffic_light,
            'overtake_allowed': opt.vehicle_base.behavior.overtake_allowed,
            'collision_time_ahead': opt.vehicle_base.behavior.collision_time_ahead,
            'overtake_counter_recover': opt.vehicle_base.behavior.overtake_counter_recover,
            'sample_resolution': opt.vehicle_base.behavior.sample_resolution,
            'local_planner': opt.vehicle_base.behavior.local_planner
        }
        
        self.controller = {
            'type': opt.vehicle_base.controller.type,
            'args': opt.vehicle_base.controller.args
        }
        
        self.v2x = {
            'enabled': opt.vehicle_base.v2x.enabled,
            'communication_range': opt.vehicle_base.v2x.communication_range
        }
        
        self.safety_manager = opt.vehicle_base.safety_manager
