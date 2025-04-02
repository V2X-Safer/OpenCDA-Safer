from opencda.scenario_testing.evaluations.evaluate_manager import EvaluationManager
from opencda.core.application.platooning.platooning_manager import PlatooningManager
from typing import Dict, List, Any, Optional, Tuple
from opencda.scenario_testing.utils.sim_api import CavWorld
import matplotlib.pyplot as plt
# import typing
from src import opt, utils_


class OracleManager(EvaluationManager):
    """
    Oracle Manager for evaluating and monitoring vehicle performance.
    
    Attributes:
        cav_world: CavWorld object
        _metrics: Dictionary containing various metrics for evaluation
    """
    
    def __init__(self, *args, **kwargs):
        """
        Initialize the Oracle Manager.
        """
        self.cav_world: CavWorld = None
        super().__init__(*args, **kwargs)

        self._metrics = {}
        
    def evaluate_following(self, ego_vehicle, leader_vehicle):
        """
        Evaluate the following performance of the ego vehicle.
        
        Args:
            ego_vehicle: The ego vehicle object
            leader_vehicle: The leader vehicle object
            
        Returns:
            score: The evaluation score
        """
        # following_metric = FollowingMetric(ego_vehicle, leader_vehicle)
        # score = following_metric.get_score()
        
        # # Store metrics for later analysis
        # if 'following_scores' not in self._metrics:
        #     self._metrics['following_scores'] = []
        # self._metrics['following_scores'].append(score)
        
        # return score
        pass
    
    def get_metrics(self):
        """
        Get the stored metrics.
        
        Returns:
            The metrics dictionary
        """
        return self._metrics
    
    def evaluate(self) -> Tuple[int, bool]:
        """
        计算oracle

        Returns:
            Tuple[int, bool]: 得分 和 是否碰撞
        """

        # fix bug in opencda
        vm = self.cav_world.get_ego_vehicle_manager()
        imu_data = vm.safety_manager.imu_sensor.imu_data
        real_route = vm.v2x_manager.ego_dynamic_trace
        timestamps = list(map(lambda e: e[2], real_route))
        while len(timestamps) < len(imu_data):
            imu_data.popleft()

        distance_oracle, joined_platoon = self.evaluate_platoon()
        status_oracle, is_collision = self.evaluate_status()
        hard_turn_oracle = self.evaluate_acc()
        # merge_time_oracle = self.evaluate_merge_time()

        # super().evaluate()
        # 归一化 oracle
        oracle = sum(len(i) for i in distance_oracle.values()) + \
                sum(len(i) for i in status_oracle.values()) + \
                sum(len(i) for i in hard_turn_oracle.values()) - \
                len(joined_platoon) * opt.platoon_joined_penalty
        if self.cav_world.get_platoon_dict():
            platoon_dict = self.cav_world.get_platoon_dict()
            for platoon_index, platoon in platoon_dict.items():
                platoon: PlatooningManager
        return oracle, is_collision
    
    
    def merge_oracle(self):
        pass
        

    def evaluate_platoon(self) -> Tuple[Dict[int, List[float]], List[PlatooningManager]]:
        """
        get distance oracle

        Returns:
            Dict[int, List[float]]: 距离oracle
        """
        platoon_dict = self.cav_world.get_platoon_dict()
        score = {}
        joined_platoon_list = []
        for platoon_index, platoon in platoon_dict.items():
            platoon: PlatooningManager  # Type annotation after variable assignment
            head_manager = platoon.vehicle_manager_list[0]
            score[platoon_index] = []
            joined_vehicle = 0
            for gnss_index, gnss_dict in enumerate(head_manager.safety_manager.sensors[4].data): 
                head_transform = gnss_dict['transformation']

                for vehicle_manager in platoon.vehicle_manager_list:
                    # 对每辆车只计入一次是否在车队内
                    if not hasattr(vehicle_manager.agent, 'added'): 
                        vehicle_manager.agent.added = False
                    if vehicle_manager.agent.joined and \
                        not vehicle_manager.agent.added:
                        joined_vehicle += 1
                        vehicle_manager.agent.added = True
                    # 跳过头车
                    if vehicle_manager == head_manager \
                        or gnss_index >= len(vehicle_manager.safety_manager.sensors[4].data): continue 

                    vehicle_transform = vehicle_manager.safety_manager.sensors[4].data[gnss_index]['transformation']

                    dis = utils_.get_distance(head_transform, vehicle_transform)
                    if dis < opt.vehicle_min_distance:
                        score[platoon_index].append(abs(dis - opt.vehicle_min_distance))
            if joined_vehicle == len(platoon.vehicle_manager_list):
                joined_platoon_list.append(platoon)

        return score, joined_platoon_list

    
    def evaluate_status(self) -> Tuple[Dict[str, List[float]], bool]:
        """
        评估状态

        Returns:
            Tuple[Dict[str, List[float]], bool]: 碰撞 停车 违反交通信号 出道 以及是否成功的标志
        """

        score = {
            'collision': [],
            'stuck': [],
            'offroad': [],
            'ran_light': []
        }
        self.status_dict: dict = self.cav_world.get_ego_vehicle_manager().safety_manager.status_queue
        for clock, status in self.status_dict:
            if status['collision']:
                score['collision'].append(True)
            if status['stuck']:
                score['stuck'].append(True)
            if status['offroad']:
                score['stuck'].append(True)
            if status['ran_light']:
                score['ran_light'].append(True)
        
        is_collision = False
        if any(score['collision']):
            is_collision = True
        return score, is_collision
        
        
    def evaluate_acc(self) -> Dict[str, List[float]]:
        """
        评估加速度相关信息
        Returns:
            Dict[str, List[float]]: 急刹车 急加速 急转弯
        """
        score = {
            'hard_brake': [],
            'hard_acc': [],
            'hard_turn': []
        }
        imu_data: list[tuple[int]] = self.cav_world.get_ego_vehicle_manager().safety_manager.imu_sensor.imu_data
        for linear_acc, angular_acc, signed_forward_acc in imu_data:
            if signed_forward_acc > opt.acc_threshold:
                score['hard_acc'].append(signed_forward_acc)
            if -signed_forward_acc > opt.brake_threshold:
                score['hard_brake'].append(linear_acc)
            if abs(angular_acc.z) > opt.angular_threshold:
                score['hard_turn'].append(angular_acc)
        return score

    
    def visualize_metrics(self):
        """
        Visualize the collected metrics.
        """
        if 'following_scores' in self._metrics and len(self._metrics['following_scores']) > 0:
            plt.figure(figsize=(10, 6))
            plt.plot(self._metrics['following_scores'])
            plt.title('Following Performance Scores')
            plt.xlabel('Time Steps')
            plt.ylabel('Score')
            plt.grid(True)
            plt.savefig('following_performance.png')
            plt.close()

