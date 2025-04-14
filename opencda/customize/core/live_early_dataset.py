"""
实时早期融合数据集类，用于从共享存储获取数据
"""
import open3d as o3d
import math
import time
import numpy as np
import torch
from collections import OrderedDict

from opencood.data_utils.datasets import COM_RANGE
from opencood.data_utils.datasets.early_fusion_dataset import EarlyFusionDataset
from opencood.utils import pcd_utils
from opencood.utils import box_utils
from opencood.utils.pcd_utils import mask_points_by_range, mask_ego_points
from opencood.utils.transformation_utils import x1_to_x2
from opencood.utils.box_utils import project_points_by_matrix_torch
from opencood.utils.box_utils import mask_boxes_outside_range_numpy

from opencda.customize.core.v2x_data_dumper import SharedDataStorage


class LiveEarlyFusionDataset(EarlyFusionDataset):
    """
    实时早期融合数据集，从SharedDataStorage中获取实时数据。
    
    Parameters
    ----------
    params : dict
        配置参数，包括预处理和后处理参数
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
        
        
    # TODO: add time delay support
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
            cav_data['ego'] = (cav_id == ego_id)
            cav_data['time_delay'] = 0
            base_data_dict[cav_id]['params'] = self.reform_param(
                cav_data,
                ego_cav_content,
                reference_frame,
                reference_frame,
                cav_id == ego_id
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
        Reform the data params with current timestamp object groundtruth and
        delay timestamp LiDAR pose for other CAVs.

        Parameters
        ----------
        cav_content : dict
            Dictionary that contains all file paths in the current cav/rsu.

        ego_content : dict
            Ego vehicle content.

        timestamp_cur : str
            The current timestamp.

        timestamp_delay : str
            The delayed timestamp.

        cur_ego_pose_flag : bool
            Whether use current ego pose to calculate transformation matrix.

        Return
        ------
        The merged parameters.
        """
        cur_params = cav_content['yaml_data']
        delay_params = cav_content['yaml_data']

        cur_ego_params = ego_content['yaml_data']
        delay_ego_params = ego_content['yaml_data']

        # we need to calculate the transformation matrix from cav to ego
        # at the delayed timestamp
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
        # This is only used for late fusion, as it did the transformation
        # in the postprocess, so we want the gt object transformation use
        # the correct one
        gt_transformation_matrix = x1_to_x2(cur_cav_lidar_pose,
                                            cur_ego_lidar_pose)

        # we always use current timestamp's gt bbx to gain a fair evaluation
        delay_params['vehicles'] = cur_params['vehicles']
        delay_params['transformation_matrix'] = transformation_matrix
        delay_params['gt_transformation_matrix'] = \
            gt_transformation_matrix
        delay_params['spatial_correction_matrix'] = spatial_correction_matrix

        return delay_params
    
    def __len__(self):
        """
        返回数据集的长度，实时数据集长度为1
        """
        return 9999999999
        
    def __getitem__(self, idx):
        base_data_dict = self.retrieve_base_data()
        if base_data_dict is None:
            return None

        processed_data_dict = OrderedDict()
        processed_data_dict['ego'] = {}

        ego_id = -1
        ego_lidar_pose = []

        # first find the ego vehicle's lidar pose
        for cav_id, cav_content in base_data_dict.items():
            if cav_content['ego']:
                ego_id = cav_id
                ego_lidar_pose = cav_content['params']['lidar_pose']
                break

        assert ego_id != -1
        assert len(ego_lidar_pose) > 0

        projected_lidar_stack = []
        object_stack = []
        object_id_stack = []

        # loop over all CAVs to process information
        for cav_id, selected_cav_base in base_data_dict.items():
            # check if the cav is within the communication range with ego
            distance = \
                math.sqrt((selected_cav_base['params']['lidar_pose'][0] -
                           ego_lidar_pose[0]) ** 2 + (
                                  selected_cav_base['params'][
                                      'lidar_pose'][1] - ego_lidar_pose[
                                      1]) ** 2)
            if distance > COM_RANGE:
                continue

            selected_cav_processed = self.get_item_single_car(
                selected_cav_base,
                ego_lidar_pose)
            # all these lidar and object coordinates are projected to ego
            # already.
            projected_lidar_stack.append(
                selected_cav_processed['projected_lidar'])
            object_stack.append(selected_cav_processed['object_bbx_center'])
            object_id_stack += selected_cav_processed['object_ids']

        # exclude all repetitive objects
        unique_indices = \
            [object_id_stack.index(x) for x in set(object_id_stack)]
        object_stack = np.vstack(object_stack)
        object_stack = object_stack[unique_indices]

        # make sure bounding boxes across all frames have the same number
        object_bbx_center = \
            np.zeros((self.params['postprocess']['max_num'], 7))
        mask = np.zeros(self.params['postprocess']['max_num'])
        object_bbx_center[:object_stack.shape[0], :] = object_stack
        mask[:object_stack.shape[0]] = 1

        # convert list to numpy array, (N, 4)
        projected_lidar_stack = np.vstack(projected_lidar_stack)

        # data augmentation
        projected_lidar_stack, object_bbx_center, mask = \
            self.augment(projected_lidar_stack, object_bbx_center, mask)

        # we do lidar filtering in the stacked lidar
        projected_lidar_stack = mask_points_by_range(projected_lidar_stack,
                                                     self.params['preprocess'][
                                                         'cav_lidar_range'])
        # augmentation may remove some of the bbx out of range
        object_bbx_center_valid = object_bbx_center[mask == 1]
        object_bbx_center_valid, range_mask = \
            box_utils.mask_boxes_outside_range_numpy(object_bbx_center_valid,
                                                     self.params['preprocess'][
                                                         'cav_lidar_range'],
                                                     self.params['postprocess'][
                                                         'order'],
                                                     return_mask=True
                                                     )
        mask[object_bbx_center_valid.shape[0]:] = 0
        object_bbx_center[:object_bbx_center_valid.shape[0]] = \
            object_bbx_center_valid
        object_bbx_center[object_bbx_center_valid.shape[0]:] = 0
        unique_indices = list(np.array(unique_indices)[range_mask])

        # pre-process the lidar to voxel/bev/downsampled lidar
        lidar_dict = self.pre_processor.preprocess(projected_lidar_stack)

        # generate the anchor boxes
        anchor_box = self.post_processor.generate_anchor_box()

        # generate targets label
        label_dict = \
            self.post_processor.generate_label(
                gt_box_center=object_bbx_center,
                anchors=anchor_box,
                mask=mask)

        processed_data_dict['ego'].update(
            {'object_bbx_center': object_bbx_center,
             'object_bbx_mask': mask,
             'object_ids': [object_id_stack[i] for i in unique_indices],
             'anchor_box': anchor_box,
             'processed_lidar': lidar_dict,
             'label_dict': label_dict})

        if self.visualize:
            processed_data_dict['ego'].update({'origin_lidar':
                                                   projected_lidar_stack})

        return processed_data_dict
