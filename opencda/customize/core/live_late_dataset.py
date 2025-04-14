"""
实时后期融合数据集类，用于从共享存储获取数据并进行后期融合处理
"""
import open3d as o3d
import math
import time
import numpy as np
import torch
from collections import OrderedDict

from opencood.data_utils.datasets import COM_RANGE
from opencood.data_utils.datasets.late_fusion_dataset import LateFusionDataset
from opencood.data_utils.post_processor import build_postprocessor
from opencood.utils import box_utils
from opencood.utils.pcd_utils import mask_points_by_range, mask_ego_points, shuffle_points
from opencood.utils.transformation_utils import x1_to_x2
from opencood.utils.box_utils import project_points_by_matrix_torch

from opencda.customize.core.v2x_data_dumper import SharedDataStorage


class LiveLateFusionDataset(LateFusionDataset):
    """
    实时后期融合数据集，从SharedDataStorage中获取实时数据并应用后期融合策略。
    
    Parameters
    ----------
    params : dict
        配置参数，包括预处理和后处理参数
    vehicle_id : str
        车辆ID，用于标识当前数据集实例
    visualize : bool
        是否可视化
    train : bool
        是否为训练模式（此类通常用于推理，因此默认为False）
    """
    def __init__(self, params, vehicle_id, visualize=True, train=False):
        super().__init__(params, visualize, train)
        
        self.vehicle_id = vehicle_id
        
        # 连接到共享数据存储
        self.shared_storage = SharedDataStorage()
        
        # 实时数据集不需要记录场景数据库
        self.scenario_database = OrderedDict()
        
    def retrieve_base_data(self, reference_frame=None, max_frame_diff=1):
        """
        从共享存储中检索实时数据，使用帧号进行同步
        
        Parameters
        ----------
        reference_frame : int, optional
            参考帧号，用于同步数据。如果为None，则使用最新帧
        max_frame_diff : int
            允许的最大帧号差异
            
        Returns
        -------
        base_data_dict : OrderedDict
            包含所有车辆实时数据的有序字典
        """
        # 获取同步数据，如果reference_frame为None，则自动使用最新数据
        if reference_frame is None:
            vehicle_ids = self.shared_storage.get_all_vehicle_ids()
            if not vehicle_ids:
                return None
            
            # 使用第一个车辆的最新帧号作为参考
            latest_data = self.shared_storage.get_latest_data(vehicle_ids[0])
            if latest_data is None or 'frame' not in latest_data:
                return None
            
            reference_frame = latest_data['frame']
        
        # 获取同步数据
        synchronized_data = self.shared_storage.get_synchronized_data_by_frame(reference_frame, max_frame_diff)
        if not synchronized_data:
            return None
        
        # 构建基础数据字典
        base_data_dict = OrderedDict()
        
        # 设置第一个车辆为ego车辆
        vehicle_ids = list(synchronized_data.keys())
        ego_id = vehicle_ids[0]  # 假设第一个是ego
        ego_cav_content = synchronized_data[ego_id]
        
        # 找到ego车辆的lidar位姿作为参考
        ego_lidar_pose = None
        for cav_id, cav_data in synchronized_data.items():
            if 'yaml_data' in cav_data and 'lidar_pose' in cav_data['yaml_data']:
                ego_lidar_pose = cav_data['yaml_data']['lidar_pose']
                break
                
        if ego_lidar_pose is None:
            return None
            
        # 遍历所有车辆数据，转换为需要的格式
        for cav_id, cav_data in synchronized_data.items():
            if 'yaml_data' not in cav_data or 'lidar_data' not in cav_data:
                continue
                
            yaml_data = cav_data['yaml_data']
            lidar_data = cav_data['lidar_data']
            
            if 'lidar_pose' not in yaml_data or 'points' not in lidar_data:
                continue
                
            cav_lidar_pose = yaml_data['lidar_pose']
            
            # 计算与ego的距离
            distance = math.sqrt(
                (cav_lidar_pose[0] - ego_lidar_pose[0]) ** 2 +
                (cav_lidar_pose[1] - ego_lidar_pose[1]) ** 2
            )
            
            # 跳过超出通信范围的车辆
            if distance > COM_RANGE and cav_id != ego_id:
                continue
                
            # 构建单车数据字典
            base_data_dict[cav_id] = OrderedDict()
            base_data_dict[cav_id]['time_delay'] = 0
            base_data_dict[cav_id]['ego'] = (cav_id == ego_id)
            
            # 转换yaml_data到需要的格式
            processed_yaml_data = yaml_data.copy()
            
            # 添加转换矩阵
            if cav_id != ego_id:
                transformation_matrix = x1_to_x2(cav_lidar_pose, ego_lidar_pose)
                processed_yaml_data['transformation_matrix'] = transformation_matrix
            
            base_data_dict[cav_id]['params'] = processed_yaml_data
            base_data_dict[cav_id]['lidar_np'] = self.pcd_to_np(cav_data['lidar_data']['points'])
            
            # 计算与ego的距离（用于后续处理）
            base_data_dict[cav_id]['distance_to_ego'] = distance
            
        return base_data_dict
        
    def pcd_to_np(self, pcd_data):
        """
        将点云数据字典转换为numpy数组格式
        
        Parameters
        ----------
        pcd_data : np.ndarray
            包含点云数据的numpy数组，形状为(n, 4)
            
        Returns
        -------
        pcd_np : np.ndarray
            处理后的点云数据，形状为(n, 4)
        """
        # 已经是numpy数组，直接使用
        points = pcd_data
        
        # 直接返回原始点云数据，确保类型为float32
        return points.astype(np.float32)
    
    def __len__(self):
        """
        返回数据集的长度，实时数据集长度为无限大
        """
        return 9999999999
        
    def __getitem__(self, idx):
        """
        获取实时数据，处理后返回
        
        Parameters
        ----------
        idx : int
            索引，在实时数据集中被忽略
            
        Returns
        -------
        processed_data_dict : dict
            处理后的数据字典
        """
        base_data_dict = self.retrieve_base_data()
        if base_data_dict is None:
            return None
            
        # 对于实时数据集，我们总是使用测试模式来处理数据
        processed_data_dict = self.get_item_test(base_data_dict)
        
        return processed_data_dict