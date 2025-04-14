from collections import OrderedDict
import numpy as np
import torch
from opencood.data_utils.datasets import COM_RANGE
from opencood.hypes_yaml import yaml_utils
from opencood.utils import box_utils
from opencood.utils.pcd_utils import mask_points_by_range


def aggregate_and_infer_from_timestamp(timestamp, root_dir, model, params, 
                                       fusion_method='early', visualize=False):
    """
    从指定时间戳聚合V2X数据并使用模型进行推理，使用OpenCOOD的原生处理方法和推理工具
    
    Parameters
    ----------
    timestamp : int
        要处理的时间戳/帧号
    root_dir : str
        数据根目录路径
    model : nn.Module
        已加载权重的模型
    params : dict
        处理参数，包含预处理和后处理配置
    fusion_method : str, optional
        融合方法，可选 'early', 'intermediate', 'late'
    visualize : bool, optional
        是否可视化结果
        
    Returns
    -------
    pred_box_tensor : torch.Tensor
        预测边界框张量
    pred_score : torch.Tensor
        预测置信度张量
    gt_box_tensor : torch.Tensor
        真实边界框张量
    base_data_dict : dict
        原始数据字典，包含每个车辆的基础数据
    """
    
    from opencood.data_utils.pre_processor import build_preprocessor
    from opencood.data_utils.post_processor import build_postprocessor
    from opencood.tools import train_utils, inference_utils
    from opencood.utils.pcd_utils import mask_points_by_range
    import os
    
    # 设置设备
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    model.to(device)
    model.eval()
    
    # 构建预处理器和后处理器
    pre_processor = build_preprocessor(params['preprocess'], train=False)
    post_processor = build_postprocessor(params['postprocess'], train=False)
    
    # 从根目录加载该时间戳的所有车辆数据
    base_data_dict = OrderedDict()
    
    # 搜索指定时间戳的数据文件
    cav_ids = [d for d in os.listdir(root_dir) if os.path.isdir(os.path.join(root_dir, d))]
    
    # 将时间戳转换为六位格式的字符串 (例如: 78 -> '000078')
    timestamp_str = str(timestamp).zfill(6)
    
    # 找出ego车辆
    ego_id = None
    ego_lidar_pose = None
    
    # 首先找出ego车辆数据
    for cav_id in cav_ids:
        cav_path = os.path.join(root_dir, cav_id)
        timestamp_files = [f for f in os.listdir(cav_path) 
                         if f.endswith('.yaml') and f.split('.')[0].isdigit()]
        
        # 检查是否存在该时间戳的文件
        matching_files = [f for f in timestamp_files if f.startswith(timestamp_str)]
        
        if matching_files:
            yaml_file = os.path.join(cav_path, matching_files[0])
            yaml_data = yaml_utils.load_yaml(yaml_file)
            
            # 检查是否是ego车辆
            if 'ego' in yaml_data and yaml_data['ego']:
                ego_id = cav_id
                ego_lidar_pose = yaml_data['lidar_pose']
                break
    
    # 如果没有找到ego车辆，则使用第一个车辆作为ego
    if ego_id is None and cav_ids:
        ego_id = cav_ids[0]
        cav_path = os.path.join(root_dir, ego_id)
        timestamp_files = [f for f in os.listdir(cav_path) 
                          if f.endswith('.yaml') and f.split('.')[0].isdigit()]
        
        matching_files = [f for f in timestamp_files if f.startswith(timestamp_str)]
        
        if matching_files:
            yaml_file = os.path.join(cav_path, matching_files[0])
            yaml_data = yaml_utils.load_yaml(yaml_file)
            ego_lidar_pose = yaml_data['lidar_pose']
    
    # 如果找不到任何车辆数据，返回None
    if ego_id is None:
        print(f"在时间戳 {timestamp_str} 中找不到任何车辆数据")
        return None, None, None, None
    
    # 加载所有车辆数据
    for cav_id in cav_ids:
        cav_path = os.path.join(root_dir, cav_id)
        timestamp_files = [f for f in os.listdir(cav_path) 
                         if f.endswith('.yaml') and f.split('.')[0].isdigit()]
        
        matching_files = [f for f in timestamp_files if f.startswith(timestamp_str)]
        
        if matching_files:
            yaml_file = os.path.join(cav_path, matching_files[0])
            lidar_file = yaml_file.replace('.yaml', '.pcd')
            
            if not os.path.exists(lidar_file):
                lidar_file = yaml_file.replace('.yaml', '.bin')
                if not os.path.exists(lidar_file):
                    continue
            
            yaml_data = yaml_utils.load_yaml(yaml_file)
            
            # 计算与ego车辆的距离
            distance = np.sqrt(
                (yaml_data['lidar_pose'][0] - ego_lidar_pose[0]) ** 2 +
                (yaml_data['lidar_pose'][1] - ego_lidar_pose[1]) ** 2
            )
            
            # 判断是否在通信范围内
            if distance > COM_RANGE and cav_id != ego_id:
                continue
                
            # 构建基础数据字典
            base_data_dict[cav_id] = OrderedDict()
            base_data_dict[cav_id]['ego'] = (cav_id == ego_id)
            
            # 加载点云数据
            if lidar_file.endswith('.pcd'):
                import open3d as o3d
                pcd = o3d.io.read_point_cloud(lidar_file)
                lidar_np = np.asarray(pcd.points)
                if len(lidar_np) > 0 and lidar_np.shape[1] == 3:
                    intensity = np.ones((lidar_np.shape[0], 1))
                    lidar_np = np.hstack((lidar_np, intensity))
            else:
                lidar_np = np.fromfile(lidar_file, dtype=np.float32).reshape(-1, 4)
            
            base_data_dict[cav_id]['lidar_np'] = lidar_np
            
            # 计算从该车辆到ego的变换矩阵
            transformation_matrix = x1_to_x2(yaml_data['lidar_pose'], ego_lidar_pose)
            yaml_data['transformation_matrix'] = transformation_matrix
            
            base_data_dict[cav_id]['params'] = yaml_data
            
            # 记录与ego的距离
            base_data_dict[cav_id]['distance_to_ego'] = distance
    
    # 如果没有数据，返回None
    if not base_data_dict:
        print(f"在时间戳 {timestamp_str} 中没有符合条件的车辆数据")
        return None, None, None, None
    
    # 处理数据并准备模型输入
    processed_data_dict = OrderedDict()
    processed_data_dict['ego'] = {}
    
    projected_lidar_stack = []
    object_stack = []
    object_id_stack = []
    
    # 对每辆车进行处理
    for cav_id, cav_content in base_data_dict.items():
        # 投影点云到ego坐标系
        lidar_np = cav_content['lidar_np']
        transformation_matrix = cav_content['params']['transformation_matrix']
        
        # 应用变换矩阵将点云转换到ego坐标系
        # 创建齐次坐标
        num_points = lidar_np.shape[0]
        homo_points = np.ones((num_points, 4))
        homo_points[:, :3] = lidar_np[:, :3]
        # 应用变换
        transformed_points = np.dot(transformation_matrix, homo_points.T).T
        lidar_np[:, :3] = transformed_points[:, :3]
        
        projected_lidar_stack.append(lidar_np)
        
        # 处理目标框数据
        if 'vehicles' in cav_content['params'] and cav_content['params']['vehicles']:
            for veh_id, vehicle in cav_content['params']['vehicles'].items():
                # 跳过自己
                if veh_id == cav_id:
                    continue
                    
                # 构建目标框中心点和尺寸
                if 'location' in vehicle and 'angle' in vehicle and 'extent' in vehicle:
                    loc = vehicle['location']
                    extent = vehicle['extent']
                    angle = vehicle['angle']
                    
                    # 构建框的格式 [x, y, z, dx, dy, dz, yaw]
                    object_box = [
                        loc[0], loc[1], loc[2],  # x, y, z
                        extent[0] * 2, extent[1] * 2, extent[2] * 2,  # dx, dy, dz
                        angle[1]  # yaw
                    ]
                    
                    # 如果不是ego车辆，需要将框转换到ego坐标系
                    if not cav_content['ego']:
                        # 先创建8个角点
                        from opencood.utils import box_utils
                        corners = box_utils.boxes_to_corners_3d(np.array([object_box]))[0]
                        # 应用变换矩阵
                        transformed_corners = box_utils.project_points_by_matrix_torch(
                            torch.from_numpy(corners).float(), 
                            torch.from_numpy(transformation_matrix).float()
                        ).numpy()
                        # 从转换后的角点重建边界框
                        transformed_box = box_utils.corner_to_center(transformed_corners)
                        object_box = transformed_box
                    
                    object_stack.append(object_box)
                    object_id_stack.append(veh_id)
    
    # 合并所有点云和目标框
    if projected_lidar_stack:
        projected_lidar_stack = np.vstack(projected_lidar_stack)
    else:
        print("没有可用的点云数据")
        return None, None, None, base_data_dict
    
    # 去除重复的目标框
    if object_stack:
        object_stack = np.array(object_stack)
        unique_ids = list(set(object_id_stack))
        unique_indices = [object_id_stack.index(x) for x in unique_ids]
        object_stack = object_stack[unique_indices]
        object_id_stack = [object_id_stack[i] for i in unique_indices]
    else:
        object_stack = np.zeros((0, 7))
        
    # 准备目标框数据
    max_num = params['postprocess']['max_num']
    object_bbx_center = np.zeros((max_num, 7))
    mask = np.zeros(max_num)
    
    # 填充实际的目标框数据
    valid_num = min(object_stack.shape[0], max_num)
    if valid_num > 0:
        object_bbx_center[:valid_num] = object_stack[:valid_num]
        mask[:valid_num] = 1
        
    # 过滤点云范围
    projected_lidar_stack = mask_points_by_range(
        projected_lidar_stack,
        params['preprocess']['cav_lidar_range']
    )
    
    # 预处理点云数据
    lidar_dict = pre_processor.preprocess(projected_lidar_stack)
    
    # 生成锚框
    anchor_box = post_processor.generate_anchor_box()
    
    # 构建batch数据
    batch_data = {
        'ego': {
            'object_bbx_center': torch.from_numpy(object_bbx_center).unsqueeze(0).to(device),
            'object_bbx_mask': torch.from_numpy(mask).unsqueeze(0).to(device),
            'object_ids': object_id_stack,
            'origin_lidar': projected_lidar_stack,
            'processed_lidar': lidar_dict,
            'anchor_box': torch.from_numpy(anchor_box).to(device),
            'transformation_matrix': torch.eye(4).to(device)
        }
    }
    
    # 将处理后的特征移到GPU
    for key, val in batch_data['ego']['processed_lidar'].items():
        if isinstance(val, torch.Tensor):
            batch_data['ego']['processed_lidar'][key] = val.to(device)
    
    # 将处理后的数据转为模型输入格式
    batch_data = train_utils.to_device(batch_data, device)
    
    # 执行推理
    with torch.no_grad():
        if fusion_method == 'early':
            pred_box_tensor, pred_score, gt_box_tensor = \
                inference_utils.inference_early_fusion(batch_data,
                                                     model,
                                                     None)
        elif fusion_method == 'late':
            pred_box_tensor, pred_score, gt_box_tensor = \
                inference_utils.inference_late_fusion(batch_data,
                                                    model,
                                                    None)
        else:  # intermediate fusion
            pred_box_tensor, pred_score, gt_box_tensor = \
                inference_utils.inference_intermediate_fusion(batch_data,
                                                           model,
                                                           None)
    
    # 可视化处理
    if visualize and pred_box_tensor is not None:
        from opencood.visualization import vis_utils
        vis_utils.visualize_single_sample_dataloader(
            pred_box_tensor, gt_box_tensor, projected_lidar_stack, None)
    
    return pred_box_tensor, pred_score, gt_box_tensor, base_data_dict


def x1_to_x2(pose1, pose2):
    """
    将坐标从pose1转换到pose2的坐标系下
    
    Parameters
    ----------
    pose1 : list
        形式为[x, y, z, roll, yaw, pitch]的位姿
    pose2 : list
        形式为[x, y, z, roll, yaw, pitch]的位姿
        
    Returns
    -------
    transformation_matrix : np.ndarray
        从pose1到pose2的4x4变换矩阵
    """
    from opencood.utils.transformation_utils import x_to_world, world_to_x
    
    # 定义一个与pose1一致的单位矩阵
    transformation_matrix = np.eye(4)
    
    # 将pose1转换到世界坐标系
    x1_to_world_matrix = x_to_world(pose1)
    # 将世界坐标系转换到pose2
    world_to_x2_matrix = world_to_x(pose2)
    
    # 组合两个变换矩阵
    transformation_matrix = np.dot(world_to_x2_matrix, x1_to_world_matrix)
    
    return transformation_matrix


def to_device(batch_dict, device):
    """
    将batch数据中的所有张量移动到指定设备
    
    Parameters
    ----------
    batch_dict : dict
        数据字典
    device : torch.device
        目标设备
        
    Returns
    -------
    batch_dict : dict
        移动到设备后的数据字典
    """
    for key, val in batch_dict.items():
        if isinstance(val, dict):
            batch_dict[key] = to_device(val, device)
        elif isinstance(val, torch.Tensor):
            batch_dict[key] = val.to(device)
        elif isinstance(val, list):
            if val and isinstance(val[0], torch.Tensor):
                batch_dict[key] = [item.to(device) for item in val]
            
    return batch_dict


def get_item_single_car(dataset, selected_cav_base, ego_pose):
    """
    处理单个车辆的数据
    
    Parameters
    ----------
    dataset : Dataset
        数据集实例
    selected_cav_base : dict
        单个车辆的原始数据
    ego_pose : list
        自车的位姿
        
    Returns
    -------
    processed_data_dict : dict
        处理后的车辆数据
    """
    selected_cav_processed = {}
    
    # 点云预处理
    lidar_np = selected_cav_base['lidar_np']
    params = selected_cav_base['params']
    
    # 执行从LiDAR坐标系到全局坐标系的变换
    lidar_pose = params['lidar_pose']
    
    # 使用 opencood 提供的正确 API 进行转换
    from opencood.utils.transformation_utils import x_to_world
    # 从lidar坐标系转换到世界坐标系
    world_transformation = x_to_world(lidar_pose)
    transformed_lidar = box_utils.project_points_by_matrix_torch(
        lidar_np[:, :3],
        world_transformation)
    
    # 从全局坐标系转换到自车坐标系
    from opencood.utils.transformation_utils import x_to_world
    ego_world_transformation = x_to_world(ego_pose)
    # 计算从世界坐标系到自车坐标系的变换
    world_to_ego = np.linalg.inv(ego_world_transformation)
    
    # 应用变换
    transformed_lidar = box_utils.project_points_by_matrix_torch(
        transformed_lidar,
        world_to_ego)
    
    # 合并回点云数据
    if lidar_np.shape[1] > 3:
        transformed_lidar_full = np.zeros((transformed_lidar.shape[0], lidar_np.shape[1]))
        transformed_lidar_full[:, :3] = transformed_lidar
        transformed_lidar_full[:, 3:] = lidar_np[:, 3:]
    else:
        transformed_lidar_full = transformed_lidar
    
    selected_cav_processed['projected_lidar'] = transformed_lidar_full
    
    # 处理标注数据（如果存在）
    if 'objects' in params:
        object_bbx_center, object_ids = [], []
        for obj in params['objects']:
            # 跳过非车辆物体（如果需要）
            if 'type' in obj and obj['type'] != 'VEHICLE':
                continue
                
            center = obj['center']
            
            # 使用相同的转换逻辑将物体中心点从车辆坐标系转换到自车坐标系
            center_world = box_utils.project_points_by_matrix_torch(
                np.array([center]),
                world_transformation)
            
            center_ego = box_utils.project_points_by_matrix_torch(
                center_world,
                world_to_ego)[0]
            
            # 处理3D边界框
            size = obj['dimension']
            yaw = obj.get('yaw', 0)
            
            object_bbx = np.array([
                center_ego[0], center_ego[1], center_ego[2],
                size[0], size[1], size[2], yaw
            ])
            object_bbx_center.append(object_bbx)
            object_ids.append(obj.get('id', len(object_ids)))
    else:
        object_bbx_center, object_ids = np.array([]), []
    
    if len(object_bbx_center):
        object_bbx_center = np.stack(object_bbx_center)
    else:
        object_bbx_center = np.zeros((0, 7))
    
    selected_cav_processed['object_bbx_center'] = object_bbx_center
    selected_cav_processed['object_ids'] = object_ids
    
    return selected_cav_processed


def get_item_single_frame(dataset, base_data_dict, ego_id, idx=None):
    """
    为OpenCOOD数据集添加的方法，处理单个时间戳的数据
    此方法需要被设置为数据集类的成员方法
    
    Parameters
    ----------
    base_data_dict : dict
        包含所有车辆数据的字典
    ego_id : str
        自车ID
    idx : int, optional
        数据索引，仅用于调用原始方法
        
    Returns
    -------
    processed_data_dict : dict
        处理后的数据字典
    """
    # 使用数据集的原生方法处理数据
    processed_data_dict = OrderedDict()
    processed_data_dict['ego'] = {}
    
    ego_lidar_pose = base_data_dict[ego_id]['params']['lidar_pose']
    
    projected_lidar_stack = []
    object_stack = []
    object_id_stack = []
    
    # 处理每个车辆的数据
    for cav_id, selected_cav_base in base_data_dict.items():
        # 检查通信范围
        distance = selected_cav_base['distance_to_ego']
        if distance > COM_RANGE and cav_id != ego_id:
            continue
        
        # 使用EarlyFusionDataset的方法处理单车数据
        selected_cav_processed = get_item_single_car(
            dataset,
            selected_cav_base,
            ego_lidar_pose)
        
        # 收集处理后的数据
        projected_lidar_stack.append(selected_cav_processed['projected_lidar'])
        object_stack.append(selected_cav_processed['object_bbx_center'])
        object_id_stack += selected_cav_processed['object_ids']
    
    # 按照EarlyFusionDataset.__getitem__的逻辑进行后续处理
    if object_id_stack:
        unique_indices = [object_id_stack.index(x) for x in set(object_id_stack)]
        object_stack = np.vstack(object_stack)
        object_stack = object_stack[unique_indices]
    else:
        object_stack = np.zeros((0, 7))
        unique_indices = []
    
    # 创建固定大小的目标框数组
    object_bbx_center = np.zeros((dataset.params['postprocess']['max_num'], 7))
    mask = np.zeros(dataset.params['postprocess']['max_num'])
    object_bbx_center[:min(object_stack.shape[0], dataset.params['postprocess']['max_num']), :] = \
        object_stack[:min(object_stack.shape[0], dataset.params['postprocess']['max_num'])]
    mask[:min(object_stack.shape[0], dataset.params['postprocess']['max_num'])] = 1
    
    # 合并点云数据
    projected_lidar_stack = np.vstack(projected_lidar_stack)
    
    # 数据增强（训练时使用，推理时可选）
    if dataset.train:
        projected_lidar_stack, object_bbx_center, mask = \
            dataset.augment(projected_lidar_stack, object_bbx_center, mask)
    
    # 过滤点云数据
    projected_lidar_stack = mask_points_by_range(
        projected_lidar_stack,
        dataset.params['preprocess']['cav_lidar_range'])
    
    # 过滤范围外的边界框
    object_bbx_center_valid = object_bbx_center[mask == 1]
    object_bbx_center_valid, range_mask = \
        box_utils.mask_boxes_outside_range_numpy(
            object_bbx_center_valid,
            dataset.params['preprocess']['cav_lidar_range'],
            dataset.params['postprocess']['order'],
            return_mask=True)
    
    # 更新目标框和掩码
    mask[np.sum(mask == 1):] = 0
    object_bbx_center[:object_bbx_center_valid.shape[0]] = object_bbx_center_valid
    object_bbx_center[object_bbx_center_valid.shape[0]:] = 0
    if unique_indices:
        unique_indices = list(np.array(unique_indices)[range_mask])
    
    # 预处理点云数据
    lidar_dict = dataset.pre_processor.preprocess(projected_lidar_stack)
    
    # 生成锚框
    anchor_box = dataset.post_processor.generate_anchor_box()
    
    # 确保target_args中有pos_threshold参数
    if 'target_args' not in dataset.params['postprocess']:
        dataset.params['postprocess']['target_args'] = {}
    if 'pos_threshold' not in dataset.params['postprocess']['target_args']:
        dataset.params['postprocess']['target_args']['pos_threshold'] = 0.6  # 设置默认值
    
    # 生成标签（训练时使用）
    label_dict = dataset.post_processor.generate_label(
        gt_box_center=object_bbx_center,
        anchors=anchor_box,
        mask=mask)
    
    # 更新处理后的数据字典
    processed_data_dict['ego'].update({
        'object_bbx_center': object_bbx_center,
        'object_bbx_mask': mask,
        'object_ids': [object_id_stack[i] for i in unique_indices] if unique_indices else [],
        'anchor_box': anchor_box,
        'processed_lidar': lidar_dict,
        'label_dict': label_dict
    })
    
    if dataset.visualize:
        processed_data_dict['ego'].update({'origin_lidar': projected_lidar_stack})
    
    return processed_data_dict


def load_config_and_model(config_path, model_path=None):
    """
    加载配置文件和模型
    
    Parameters
    ----------
    config_path : str
        配置文件路径
    model_path : str, optional
        模型权重文件路径
        
    Returns
    -------
    model : nn.Module
        加载好的模型
    params : dict
        加载好的配置参数
    """
    import torch
    from opencood.tools import train_utils
    
    # 加载配置文件
    params = yaml_utils.load_yaml(config_path)
    
    # 创建模型实例
    model = train_utils.create_model(params)
    
    # 如果提供了权重路径，加载预训练权重
    if model_path:
        _, model = train_utils.load_saved_model(model_path, model)
    
    # 确保模型处于评估模式
    model.eval()
    
    return model, params


import yaml
import torch
from opencood.tools import train_utils

# 加载配置文件
config_path = '/home/test/V2X/OpenCDA/OpenCDA/opencood/model_dir/pointpillar_early_fusion/config.yaml'
params = yaml_utils.load_yaml(config_path)

# 加载模型
model = train_utils.create_model(params)
model_path = '/home/test/V2X/OpenCDA/OpenCDA/opencood/model_dir/pointpillar_early_fusion/latest.pth'
model = train_utils.load_saved_model(model_path, model)[1]

# 指定时间戳和数据路径
timestamp = 76  # 对应000078.yaml和000078.pcd
root_dir = '/home/test/V2X/OpenCDA/OpenCDA/data_dumping/test_culver_city/2025_04_14_10_46_39'

# 执行聚合和推理
pred_boxes, pred_scores, gt_boxes, base_data = \
    aggregate_and_infer_from_timestamp(timestamp, root_dir, model, params, 
                                       fusion_method='early', visualize=True)

# 使用预测结果
print(f"检测到 {len(pred_boxes)} 个物体")