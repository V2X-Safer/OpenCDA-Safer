"""
实时中间融合数据集类，用于从共享存储获取数据并进行中间特征融合
"""
import open3d as o3d
import math
import time
import numpy as np
import torch
from collections import OrderedDict

from opencood.data_utils.datasets import COM_RANGE
from opencood.data_utils.datasets.intermediate_fusion_dataset import IntermediateFusionDataset
import opencood.data_utils.post_processor as post_processor
from opencood.utils import box_utils
from opencood.utils.pcd_utils import mask_points_by_range, mask_ego_points, shuffle_points
from opencood.utils.transformation_utils import x1_to_x2
from opencood.utils.box_utils import project_points_by_matrix_torch

from opencda.customize.core.v2x_data_dumper import SharedDataStorage


class LiveIntermediateFusionDataset(IntermediateFusionDataset):
    """
    实时中间融合数据集，从SharedDataStorage中获取实时数据，并使用中间融合方式处理。
    
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
        
        # 最大CAV数量
        self.max_cav = params['train_params']['max_cav']
        
    def retrieve_base_data(self, reference_frame=None, max_frame_diff=1, cur_ego_pose_flag=True):
        """
        从共享存储中检索实时数据，使用帧号进行同步
        
        Parameters
        ----------
        reference_frame : int, optional
            参考帧号，用于同步数据。如果为None，则使用最新帧
        max_frame_diff : int
            允许的最大帧号差异
        cur_ego_pose_flag : bool
            是否使用当前ego位姿计算变换矩阵
            
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
            cav_data['ego'] = (cav_id == ego_id)
            cav_data['time_delay'] = 0
            base_data_dict[cav_id]['params'] = self.reform_param(
                cav_data,
                ego_cav_content,
                reference_frame,
                reference_frame,
                cur_ego_pose_flag
            )
            
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
        
        # 分离xyz和强度值
        xyz = points[:, :3]
        intensity = points[:, 3:]
        
        # 对于可视化目的，创建o3d点云对象（虽然此处不返回）
        point_intensity = np.c_[
            intensity,
            np.zeros_like(intensity),
            np.zeros_like(intensity)
        ]
        
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(xyz)
        pcd.colors = o3d.utility.Vector3dVector(point_intensity)
        
        # 直接返回原始点云数据，确保类型为float32
        return points.astype(np.float32)
        
    def reform_param(self, cav_content, ego_content, timestamp_cur,
                     timestamp_delay, cur_ego_pose_flag):
        """
        使用当前时间戳的目标真值和延迟时间戳的LiDAR位姿重新组织数据参数。

        Parameters
        ----------
        cav_content : dict
            包含当前车辆所有文件路径的字典。

        ego_content : dict
            自车内容。

        timestamp_cur : str
            当前时间戳。

        timestamp_delay : str
            延迟的时间戳。

        cur_ego_pose_flag : bool
            是否使用当前自车位姿计算变换矩阵。

        Returns
        -------
        合并后的参数字典。
        """
        cur_params = cav_content['yaml_data']
        delay_params = cav_content['yaml_data']

        cur_ego_params = ego_content['yaml_data']
        delay_ego_params = ego_content['yaml_data']

        # 在延迟时间戳下计算从当前车辆到自车的变换矩阵
        delay_cav_lidar_pose = delay_params['lidar_pose']
        delay_ego_lidar_pose = delay_ego_params["lidar_pose"]

        cur_ego_lidar_pose = cur_ego_params['lidar_pose']
        cur_cav_lidar_pose = cur_params['lidar_pose']

        if not cav_content['ego'] and self.loc_err_flag:
            delay_cav_lidar_pose = self.add_loc_noise(delay_cav_lidar_pose,
                                                      self.xyz_noise_std,
                                                      self.ryp_noise_std)
            cur_cav_lidar_pose = self.add_loc_noise(cur_cav_lidar_pose,
                                                    self.xyz_noise_std,
                                                    self.ryp_noise_std)

        if cur_ego_pose_flag:
            transformation_matrix = x1_to_x2(delay_cav_lidar_pose,
                                             cur_ego_lidar_pose)
            spatial_correction_matrix = np.eye(4)
        else:
            transformation_matrix = x1_to_x2(delay_cav_lidar_pose,
                                             delay_ego_lidar_pose)
            spatial_correction_matrix = x1_to_x2(delay_ego_lidar_pose,
                                                 cur_ego_lidar_pose)
        
        # 这仅用于后期融合，因为它在后处理中进行了变换，
        # 所以我们希望gt对象变换使用正确的变换
        gt_transformation_matrix = x1_to_x2(cur_cav_lidar_pose,
                                            cur_ego_lidar_pose)

        # 我们总是使用当前时间戳的gt边界框来获得公平的评估
        delay_params['vehicles'] = cur_params['vehicles']
        delay_params['transformation_matrix'] = transformation_matrix
        delay_params['gt_transformation_matrix'] = \
            gt_transformation_matrix
        delay_params['spatial_correction_matrix'] = spatial_correction_matrix

        # 为IntermediateFusionDataset类添加速度
        delay_params['ego_speed'] = cur_params.get('ego_speed', 0)

        return delay_params
    
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
        base_data_dict = self.retrieve_base_data(cur_ego_pose_flag=self.cur_ego_pose_flag)
        if base_data_dict is None:
            return None

        processed_data_dict = OrderedDict()
        processed_data_dict['ego'] = {}

        ego_id = -1
        ego_lidar_pose = []

        # 首先找到自车的雷达位姿
        for cav_id, cav_content in base_data_dict.items():
            if cav_content['ego']:
                ego_id = cav_id
                ego_lidar_pose = cav_content['params']['lidar_pose']
                break
        
        # 确保找到了自车
        if ego_id == -1 or len(ego_lidar_pose) == 0:
            return None

        # 计算两两之间的变换矩阵
        pairwise_t_matrix = self.get_pairwise_transformation(base_data_dict, self.max_cav)

        processed_features = []
        object_stack = []
        object_id_stack = []

        # 用于时间延迟校正和指示数据类型(V2V vs V2I)的先验知识
        velocity = []
        time_delay = []
        infra = []
        spatial_correction_matrix = []

        if self.visualize:
            projected_lidar_stack = []

        # 遍历所有车辆处理信息
        for cav_id, selected_cav_base in base_data_dict.items():
            # 检查车辆是否在与自车的通信范围内
            distance = math.sqrt(
                (selected_cav_base['params']['lidar_pose'][0] - ego_lidar_pose[0]) ** 2 + 
                (selected_cav_base['params']['lidar_pose'][1] - ego_lidar_pose[1]) ** 2
            )
            if distance > COM_RANGE and cav_id != ego_id:
                continue

            selected_cav_processed = self.get_item_single_car(
                selected_cav_base,
                ego_lidar_pose
            )

            object_stack.append(selected_cav_processed['object_bbx_center'])
            object_id_stack += selected_cav_processed['object_ids']
            processed_features.append(selected_cav_processed['processed_features'])

            velocity.append(selected_cav_processed['velocity'])
            time_delay.append(float(selected_cav_base['time_delay']))
            spatial_correction_matrix.append(
                selected_cav_base['params']['spatial_correction_matrix']
            )
            infra.append(1 if int(cav_id) < 0 else 0)

            if self.visualize:
                projected_lidar_stack.append(selected_cav_processed['projected_lidar'])

        # 去除所有重复的目标
        if not object_id_stack:
            # 如果没有检测到目标，创建空的目标集
            object_stack = np.zeros((0, 7))
            unique_indices = []
        else:
            unique_indices = [object_id_stack.index(x) for x in set(object_id_stack)]
            object_stack = np.vstack(object_stack)
            object_stack = object_stack[unique_indices]

        # 确保所有帧的边界框具有相同的数量
        object_bbx_center = np.zeros((self.params['postprocess']['max_num'], 7))
        mask = np.zeros(self.params['postprocess']['max_num'])
        
        if object_stack.shape[0] > 0:
            object_bbx_center[:object_stack.shape[0], :] = object_stack
            mask[:object_stack.shape[0]] = 1

        # 将不同车辆的预处理特征合并到同一字典中
        cav_num = len(processed_features)
        if cav_num > 0:
            merged_feature_dict = self.merge_features_to_dict(processed_features)
        else:
            # 处理没有车辆数据的情况
            merged_feature_dict = OrderedDict()

        # 生成锚点框
        anchor_box = self.post_processor.generate_anchor_box()

        # 生成目标标签
        label_dict = self.post_processor.generate_label(
            gt_box_center=object_bbx_center,
            anchors=anchor_box,
            mask=mask
        )

        # 将 dv, dt, infra 填充到 max_cav
        velocity = velocity + (self.max_cav - len(velocity)) * [0.]
        time_delay = time_delay + (self.max_cav - len(time_delay)) * [0.]
        infra = infra + (self.max_cav - len(infra)) * [0.]
        
        if len(spatial_correction_matrix) > 0:
            spatial_correction_matrix = np.stack(spatial_correction_matrix)
            padding_eye = np.tile(np.eye(4)[None], (self.max_cav - len(spatial_correction_matrix), 1, 1))
            spatial_correction_matrix = np.concatenate([spatial_correction_matrix, padding_eye], axis=0)
        else:
            spatial_correction_matrix = np.tile(np.eye(4)[None], (self.max_cav, 1, 1))

        processed_data_dict['ego'].update({
            'object_bbx_center': object_bbx_center,
            'object_bbx_mask': mask,
            'object_ids': [object_id_stack[i] for i in unique_indices] if unique_indices else [],
            'anchor_box': anchor_box,
            'processed_lidar': merged_feature_dict,
            'label_dict': label_dict,
            'cav_num': cav_num,
            'velocity': velocity,
            'time_delay': time_delay,
            'infra': infra,
            'spatial_correction_matrix': spatial_correction_matrix,
            'pairwise_t_matrix': pairwise_t_matrix
        })

        # 可视化时，添加原始点云
        if self.visualize and projected_lidar_stack:
            processed_data_dict['ego'].update({
                'origin_lidar': np.vstack(projected_lidar_stack)
            })

        return processed_data_dict