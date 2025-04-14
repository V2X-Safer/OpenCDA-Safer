import copy
import enum
import time
import numpy as np
import torch
import cv2
import os
import weakref
import threading
from torch.utils.data import DataLoader
from opencda.core.sensing.perception.perception_manager import PerceptionManager
from opencda.core.sensing.perception.obstacle_vehicle import ObstacleVehicle
from opencda.customize.core import live_early_dataset
from opencda.customize.core.v2x_data_dumper import SharedDataStorage

# 导入OpenCOOD相关模块
import opencood.hypes_yaml.yaml_utils as yaml_utils
from opencood.tools import train_utils, inference_utils
from opencood.data_utils.datasets import build_dataset, early_fusion_dataset
from opencood.data_utils.pre_processor import build_preprocessor
from opencood.data_utils.post_processor import build_postprocessor
from opencood.utils import eval_utils, box_utils
from opencood.utils.pcd_utils import mask_points_by_range, mask_ego_points
from src.log import log_process_critical, log_process_info

class V2XPerceptionManager(PerceptionManager):
    """
    扩展的感知管理器，支持V2X融合，使用OpenCOOD的融合方法
    
    Parameters
    ----------
    vehicle : carla.Vehicle
        Vehicle对象
    config_yaml : dict
        配置字典
    cav_world : opencda object
        CAV世界对象
    data_dump : bool
        是否保存数据
    carla_world : carla.world
        CARLA世界对象
    infra_id : int
        基础设施ID
    v2x_configs : dict
        V2X相关配置
    """
    def __init__(self, v2x_configs=None):
        self.vis = None
        self.v2x_fusion_enabled = v2x_configs.v2x
        self.model_dir = v2x_configs.model_dir
        self.confidence_threshold = v2x_configs.confidence_threshold
        self.show_v2x_detection = v2x_configs.v2x_visualize
        self.shared_storage = SharedDataStorage()
        self.fusion_method = v2x_configs.fusion_method \
            if type(v2x_configs.fusion_method) == str else 'early'

        v2x_configs.params = yaml_utils.load_yaml(
            os.path.join(self.model_dir, 'config.yaml')
        )

        if self.v2x_fusion_enabled:
            self.pre_processor = build_preprocessor(v2x_configs.params['preprocess'], train=False)
            self.post_processor = build_postprocessor(v2x_configs.params['postprocess'], train=False)
        else:
            self.pre_processor = None
            self.post_processor = None
        
        del v2x_configs.params
        
        
    def _init_v2x_fusion(self, vehicle):
        """初始化V2X融合相关的模型和数据集"""
        try:
            
            hypes = yaml_utils.load_yaml(
                os.path.join(self.model_dir, 'config.yaml')
            )
                
            # 创建数据集
            self.v2x_dataset = live_early_dataset.LiveEarlyFusionDataset(hypes, vehicle.id)
            
            # 创建模型
            self.model = train_utils.create_model(hypes)
            
            _, self.model = train_utils.load_saved_model(self.model_dir, self.model)
            
            device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
            self.model = self.model.to(device)
            self.model.eval()
            
            log_process_info(f'{str(vehicle.id)} v2x model init success')
            
        except Exception as e:
            log_process_critical('v2x model init failed')
            self.model = None
            self.v2x_dataset = None
            self.v2x_fusion_enabled = False
            
            
    def detect(self, ego_pos):
        """
        实时检测周围物体，支持V2X融合（不使用DataLoader）
        """
        # 首先调用基类的detect方法获取原始检测结果
        objects = super(V2XPerceptionManager, self).detect(ego_pos)
        
        # 如果V2X融合未启用或模型未成功加载，直接返回原始结果
        if not self.v2x_fusion_enabled or self.model is None or self.pre_processor is None:
            return objects
        
        try:
            # 直接从数据集获取数据（使用索引0或任意固定值）
            batch_data = self.v2x_dataset.__getitem__(0)
            if batch_data is None: return objects
            
            # 手动将单个样本转换为批处理格式
            if self.v2x_dataset.collate_batch_test is not None:
                batch_data = self.v2x_dataset.collate_batch_test([batch_data])
            else:
                # 如果没有collate_fn，手动创建批处理格式
                for key in batch_data['ego']:
                    if isinstance(batch_data['ego'][key], torch.Tensor):
                        # 添加批处理维度
                        batch_data['ego'][key] = batch_data['ego'][key].unsqueeze(0)
            
            frame = None
            vehicle_ids = self.shared_storage.get_all_vehicle_ids()
            if vehicle_ids:
                frame = self.shared_storage.get_latest_data(vehicle_ids[0]).get('frame')
            v2x_vehicles = []
            
            if self.shared_storage.get_result(frame):
                v2x_vehicles = self.shared_storage.get_result(frame)
                
            # 如果没有缓存的结果，则准备实时数据
            if batch_data and v2x_vehicles == []:
                v2x_vehicles = self.run_v2x_fusion(batch_data)

                # 将结果存储到共享存储中
                self.shared_storage.add_result(frame, (v2x_vehicles))
                    
            # 合并或替换检测结果
            if v2x_vehicles:
                fusion_mode = self.fusion_method
                
                if fusion_mode == 'replace':
                    objects['vehicles'] = v2x_vehicles
                else:  # 'merge'模式
                    if 'vehicles' in objects:
                        combined_vehicles = objects['vehicles'] + v2x_vehicles
                        objects['vehicles'] = self.adaptive_merge_detections(combined_vehicles)
                    else:
                        objects['vehicles'] = v2x_vehicles
                        
        except Exception as e:
            print(f"V2X融合失败，使用默认感知结果: {e}")
            import traceback
            traceback.print_exc()
        
        return objects

        
    def run_v2x_fusion(self, batch_data):
        """
        运行V2X融合算法
        
        Parameters
        ----------
        batch_data : dict
            模型输入数据
            
        Returns
        -------
        v2x_vehicles : list
            ObstacleVehicle对象列表
        """
        try:
            if self.model is None:
                print("模型未初始化，无法进行融合")
                return []
            
            # 检查CUDA是否可用
            device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
            
            # 确保模型在正确的设备上
            self.model = self.model.to(device)
            
            batch_data = train_utils.to_device(batch_data, device)
            
            with torch.no_grad():
                if self.fusion_method == 'early':
                    pred_box_tensor, pred_score, gt_box_tensor = \
                        inference_utils.inference_early_fusion(
                            batch_data, self.model,
                            self.v2x_dataset)
                elif self.fusion_method == 'late':
                    pred_box_tensor, pred_score, gt_box_tensor = \
                        inference_utils.inference_late_fusion(
                            batch_data, self.model,
                            self.v2x_dataset)
                else:  # intermediate fusion
                    pred_box_tensor, pred_score, gt_box_tensor = \
                        inference_utils.inference_intermediate_fusion(
                            batch_data, self.model, 
                            self.v2x_dataset)
                
                # 将检测结果转换为车辆对象列表
                v2x_vehicles = self.convert_detection_to_obstacles(
                    pred_box_tensor, pred_score, 
                    confidence_threshold=self.confidence_threshold)
                
                return v2x_vehicles
            
        except Exception as e:
            print(f"V2X融合运行错误: {e}")
            import traceback
            traceback.print_exc()
            return []
    
      # 返回单位矩阵作为默认值
    
    
    
    
    
    def _calculate_3d_iou(self, box1, box2):
        """
        计算两个3D边界框的IoU（简化版，仅在BEV平面计算）
        
        Parameters
        ----------
        box1, box2 : np.ndarray
            边界框参数 [x, y, z, dx, dy, dz, yaw]
            
        Returns
        -------
        iou : float
            IoU值
        """
        try:
            # 提取BEV平面上的参数
            x1, y1 = box1[0], box1[1]
            w1, l1 = box1[3], box1[4]
            
            x2, y2 = box2[0], box2[1]
            w2, l2 = box2[3], box2[4]
            
            # 简化：忽略旋转，使用AABB进行IoU计算
            # 计算AABB坐标
            x1_min, x1_max = x1 - w1/2, x1 + w1/2
            y1_min, y1_max = y1 - l1/2, y1 + l1/2
            
            x2_min, x2_max = x2 - w2/2, x2 + w2/2
            y2_min, y2_max = y2 - l2/2, y2 + l2/2
            
            # 计算交集区域
            x_overlap = max(0, min(x1_max, x2_max) - max(x1_min, x2_min))
            y_overlap = max(0, min(y1_max, y2_max) - max(y1_min, y2_min))
            intersection = x_overlap * y_overlap
            
            # 计算并集
            area1 = w1 * l1
            area2 = w2 * l2
            union = area1 + area2 - intersection
            
            # 防止除零错误
            if union < 1e-6:
                return 0
                
            # 计算IoU
            iou = intersection / union
            
            return iou
        except Exception as e:
            print(f"计算IoU出错: {e}")
            return 0
    
    def _create_corners_3d(self, x, y, z, dx, dy, dz, yaw) -> np.ndarray:
        """
        根据中心点、尺寸和朝向角创建3D边界框的8个角点
        
        Parameters
        ----------
        x, y, z : float
            边界框中心点坐标
        dx, dy, dz : float
            边界框的尺寸
        yaw : float
            偏航角（弧度）
            
        Returns
        -------
        corners : np.ndarray
            8个角点的坐标，形状为(8,3)
        """
        # 创建未旋转的边界框顶点
        corners = np.array([
            [dx/2, dy/2, dz/2],
            [dx/2, dy/2, -dz/2],
            [dx/2, -dy/2, dz/2],
            [dx/2, -dy/2, -dz/2],
            [-dx/2, dy/2, dz/2],
            [-dx/2, dy/2, -dz/2],
            [-dx/2, -dy/2, dz/2],
            [-dx/2, -dy/2, -dz/2]
        ])
        
        # 创建旋转矩阵
        rot_matrix = np.array([
            [np.cos(yaw), -np.sin(yaw), 0],
            [np.sin(yaw), np.cos(yaw), 0],
            [0, 0, 1]
        ])
        
        # 应用旋转
        corners = corners @ rot_matrix.T
        
        # 平移
        corners[:, 0] += x
        corners[:, 1] += y
        corners[:, 2] += z
        
        return corners
    
    def convert_detection_to_obstacles(self, pred_box_tensor, pred_score, confidence_threshold=0.3):
        """
        将模型检测结果转换为ObstacleVehicle对象列表
        支持两种格式的预测框:
        1. [x, y, z, dx, dy, dz, yaw] 形式的参数化表示
        2. 8个角点坐标形式的表示 (8, 3)
        
        Parameters
        ----------
        pred_box_tensor : torch.Tensor
            预测边界框张量
        pred_score : torch.Tensor
            预测置信度张量
        confidence_threshold : float
            置信度阈值
            
        Returns
        -------
        obstacle_list : list
            ObstacleVehicle对象列表
        """
        obstacle_list = []
        
        # 检查输入是否有效
        if pred_box_tensor is None or (isinstance(pred_box_tensor, torch.Tensor) and pred_box_tensor.numel() == 0):
            return obstacle_list
            
        # 将张量转换为numpy数组
        if isinstance(pred_box_tensor, torch.Tensor):
            pred_box_tensor = pred_box_tensor.cpu().numpy()
        if isinstance(pred_score, torch.Tensor):
            pred_score = pred_score.cpu().numpy()
        
        # 根据置信度阈值过滤
        if pred_score is not None and len(pred_score) > 0:
            mask = pred_score >= confidence_threshold
            pred_box_tensor = pred_box_tensor[mask]
            pred_score = pred_score[mask]
        
        # 如果没有检测到物体，返回空列表
        if pred_box_tensor is None or len(pred_box_tensor) == 0:
            return obstacle_list
        
        try:
            import open3d as o3d
        except ImportError:
            print("警告: open3d未安装，无法创建3D边界框")
            return obstacle_list
        
        # 判断输入边界框的形状，处理不同格式的边界框
        box_shape = pred_box_tensor.shape
        is_corners_format = len(box_shape) >= 2 and box_shape[-2:] == (8, 3)
        
        # 创建ObstacleVehicle对象
        for i in range(len(pred_box_tensor)):
            try:
                if is_corners_format:
                    # 当输入是角点格式时
                    corners = pred_box_tensor[i]  # 形状为(8, 3)
                    
                    # 从角点计算中心点和尺寸
                    center = np.mean(corners, axis=0)  # 中心点
                    x, y, z = center
                    
                    # 计算朝向和尺寸
                    # 假设角点顺序是标准的，我们可以通过前四个点确定朝向
                    front_center = np.mean(corners[:4], axis=0)
                    direction = front_center - center
                    yaw = np.arctan2(direction[1], direction[0])
                    
                    # 计算尺寸
                    min_coord = np.min(corners, axis=0)
                    max_coord = np.max(corners, axis=0)
                    dx, dy, dz = max_coord - min_coord
                else:
                    # 当输入是参数化格式时 [x, y, z, dx, dy, dz, yaw]
                    box = pred_box_tensor[i]
                    x, y, z, dx, dy, dz, yaw = box
                    
                    # 计算8个角点坐标
                    corners = self._create_corners_3d(x, y, z, dx, dy, dz, yaw)
                
                # 创建AABB包围盒
                o3d_points = o3d.utility.Vector3dVector(corners)
                aabb = o3d.geometry.AxisAlignedBoundingBox.create_from_points(o3d_points)
                aabb.color = (1, 0, 0)  # 红色表示V2X检测结果
                
                # 创建ObstacleVehicle对象
                score = pred_score[i] if pred_score is not None and i < len(pred_score) else 1.0
                obstacle = ObstacleVehicle(corners \
                    if corners.dtype == np.float64 \
                    else np.array(corners, dtype=np.float64), 
                    aabb)
                
                # 添加V2X特有的属性
                obstacle.v2x_score = score
                obstacle.v2x_detected = True
                obstacle.type_id = "vehicle.v2x.detected"  # 添加类型标识
                
                obstacle_list.append(obstacle)
                
            except Exception as e:
                print(f"创建障碍物对象失败 (边界框 #{i}): {e}")
                import traceback
                traceback.print_exc()
                continue
        
        return obstacle_list
    
    def adaptive_merge_detections(self, detection_list, iou_threshold=0.3):
        """
        自适应合并检测结果，根据检测质量动态调整
        
        Parameters
        ----------
        detection_list : list
            ObstacleVehicle对象列表
        iou_threshold : float
            IoU阈值，超过此阈值的框将被合并
            
        Returns
        -------
        merged_list : list
            合并后的ObstacleVehicle对象列表
        """
        if len(detection_list) <= 1:
            return detection_list
        
        # 提取所有边界框，并检查是否有V2X来源的检测
        boxes = []
        scores = []
        v2x_flags = []
        detection_indices = []
        
        for i, obstacle in enumerate(detection_list):
            if hasattr(obstacle, 'bounding_box') and obstacle.bounding_box:
                try:
                    cx = obstacle.bounding_box.location.x
                    cy = obstacle.bounding_box.location.y
                    cz = obstacle.bounding_box.location.z
                    dx = obstacle.bounding_box.extent.x * 2
                    dy = obstacle.bounding_box.extent.y * 2
                    dz = obstacle.bounding_box.extent.z * 2
                    
                    if hasattr(obstacle, 'get_transform') and obstacle.get_transform() is not None:
                        transform = obstacle.get_transform()
                        yaw = transform.rotation.yaw if hasattr(transform, 'rotation') else 0
                    else:
                        yaw = 0
                        
                    boxes.append([cx, cy, cz, dx, dy, dz, yaw])
                    detection_indices.append(i)
                    
                    # 检查是否为V2X检测并提取分数
                    is_v2x = hasattr(obstacle, 'v2x_detected') and obstacle.v2x_detected
                    v2x_flags.append(is_v2x)
                    
                    if is_v2x and hasattr(obstacle, 'v2x_score'):
                        scores.append(obstacle.v2x_score)
                    else:
                        scores.append(0.5)  # 默认分数
                except Exception as e:
                    print(f"处理障碍物属性失败: {e}")
        
        # 如果没有有效边界框，返回原列表
        if not boxes:
            return detection_list
            
        boxes = np.array(boxes)
        scores = np.array(scores)
        v2x_flags = np.array(v2x_flags)
        detection_indices = np.array(detection_indices)
        
        # 根据分数排序
        sorted_indices = np.argsort(-scores)
        boxes = boxes[sorted_indices]
        scores = scores[sorted_indices]
        v2x_flags = v2x_flags[sorted_indices]
        detection_indices = detection_indices[sorted_indices]
        
        # 应用NMS
        keep_indices = []
        for i in range(len(boxes)):
            if v2x_flags[i]:  # 如果是V2X检测结果，给予更高优先级
                keep = True
                for j in keep_indices:
                    if self._calculate_3d_iou(boxes[i], boxes[j]) > iou_threshold:
                        # 如果已保留的框也是V2X检测，则比较分数
                        if v2x_flags[j]:
                            if scores[i] <= scores[j]:
                                keep = False
                                break
                        else:
                            # 如果已保留的不是V2X检测，V2X检测优先
                            continue
                if keep:
                    keep_indices.append(i)
            else:
                # 普通检测结果使用标准NMS
                keep = True
                for j in keep_indices:
                    if self._calculate_3d_iou(boxes[i], boxes[j]) > iou_threshold:
                        keep = False
                        break
                if keep:
                    keep_indices.append(i)
        
        # 返回保留的检测结果
        return [detection_list[detection_indices[i]] for i in keep_indices]
    
    def _create_bbox_vertices_and_lines(self, box):
        """
        创建3D边界框的顶点和线段
        
        Parameters
        ----------
        box : np.ndarray
            边界框参数 [x, y, z, dx, dy, dz, yaw]
            
        Returns
        -------
        vertices : np.ndarray
            8个顶点的坐标
        lines : np.ndarray
            12条边的索引
        """
        # 提取参数
        x, y, z, dx, dy, dz, yaw = box
        
        # 创建未旋转的边界框顶点
        vertices = np.array([
            [dx/2, dy/2, dz/2],
            [dx/2, dy/2, -dz/2],
            [dx/2, -dy/2, dz/2],
            [dx/2, -dy/2, -dz/2],
            [-dx/2, dy/2, dz/2],
            [-dx/2, dy/2, -dz/2],
            [-dx/2, -dy/2, dz/2],
            [-dx/2, -dy/2, -dz/2]
        ])
        
        # 创建旋转矩阵
        rot_matrix = np.array([
            [np.cos(yaw), -np.sin(yaw), 0],
            [np.sin(yaw), np.cos(yaw), 0],
            [0, 0, 1]
        ])
        
        # 应用旋转
        vertices = vertices @ rot_matrix.T
        
        # 应用平移
        vertices[:, 0] += x
        vertices[:, 1] += y
        vertices[:, 2] += z
        
        # 定义边界框的12条边
        lines = np.array([
            [0, 1], [0, 2], [1, 3], [2, 3],
            [4, 5], [4, 6], [5, 7], [6, 7],
            [0, 4], [1, 5], [2, 6], [3, 7]
        ])
        
        return vertices, lines
        
    def visualize_realtime_fusion(self, pred_box_tensor, gt_box_tensor, origin_lidar):
        """
        可视化实时融合结果
        
        Parameters
        ----------
        pred_box_tensor : torch.Tensor
            预测边界框
        gt_box_tensor : torch.Tensor
            真实边界框
        origin_lidar : torch.Tensor
            原始点云数据
        """
        if not self.show_v2x_detection:
            return
            
        try:
            import open3d as o3d
            
            # 初始化可视化窗口
            if not hasattr(self, 'vis') or self.vis is None:
                self.vis = o3d.visualization.Visualizer()
                self.vis.create_window("V2X Fusion Visualization", width=1280, height=720)
                
                # 设置可视化参数
                render_option = self.vis.get_render_option()
                render_option.background_color = [0.05, 0.05, 0.05]
                render_option.point_size = 1.0
                render_option.show_coordinate_frame = True
                
                # 初始化点云和边界框对象
                self.vis_pcd = o3d.geometry.PointCloud()
                self.vis.add_geometry(self.vis_pcd)
                
                # 初始化预测框和真实框对象
                self.pred_bbox_lineset = []
                self.gt_bbox_lineset = []
                
                max_boxes = 50  # 支持最多50个框
                for i in range(max_boxes):
                    pred_lineset = o3d.geometry.LineSet()
                    gt_lineset = o3d.geometry.LineSet()
                    self.pred_bbox_lineset.append(pred_lineset)
                    self.gt_bbox_lineset.append(gt_lineset)
                    self.vis.add_geometry(pred_lineset)
                    self.vis.add_geometry(gt_lineset)
                
                # 添加视角控制
                view_control = self.vis.get_view_control()
                params = view_control.convert_to_pinhole_camera_parameters()
                params.extrinsic = np.array([
                    [0, -1, 0, 0],
                    [0, 0, -1, -30],
                    [1, 0, 0, 0],
                    [0, 0, 0, 1]
                ])
                view_control.convert_from_pinhole_camera_parameters(params)
            
            # 更新点云数据
            if origin_lidar is not None and origin_lidar.shape[0] > 0:
                if isinstance(origin_lidar, torch.Tensor):
                    origin_lidar = origin_lidar.cpu().numpy()
                    
                # 更新点云
                self.vis_pcd.points = o3d.utility.Vector3dVector(origin_lidar[:, :3])
                
                # 使用intensity作为颜色
                if origin_lidar.shape[1] >= 4:
                    intensity = origin_lidar[:, 3]
                    colors = np.zeros((intensity.shape[0], 3))
                    
                    # 将intensity归一化到[0,1]
                    min_val = np.min(intensity)
                    max_val = np.max(intensity)
                    if max_val > min_val:
                        intensity_norm = (intensity - min_val) / (max_val - min_val)
                    else:
                        intensity_norm = np.zeros_like(intensity)
                    
                    # 使用热力图颜色映射
                    colors[:, 0] = intensity_norm  # R
                    colors[:, 1] = intensity_norm * 0.5  # G
                    colors[:, 2] = 1 - intensity_norm  # B
                    
                    self.vis_pcd.colors = o3d.utility.Vector3dVector(colors)
                    
                self.vis.update_geometry(self.vis_pcd)
            
            # 更新预测边界框
            if pred_box_tensor is not None:
                if isinstance(pred_box_tensor, torch.Tensor):
                    pred_box_tensor = pred_box_tensor.cpu().numpy()
                if len(pred_box_tensor.shape) == 3 and pred_box_tensor.shape[0] == 1:
                    # 处理批处理维度
                    pred_box_tensor = pred_box_tensor[0]
                
                # 清除旧边界框
                for lineset in self.pred_bbox_lineset:
                    lineset.points = o3d.utility.Vector3dVector([])
                    lineset.lines = o3d.utility.Vector2iVector([])
                    lineset.colors = o3d.utility.Vector3dVector([])
                    self.vis.update_geometry(lineset)
                
                # 绘制新边界框
                for i, box in enumerate(pred_box_tensor):
                    if i >= len(self.pred_bbox_lineset):
                        break
                        
                    # 创建边界框顶点和线
                    vertices, lines = self._create_bbox_vertices_and_lines(box)
                    self.pred_bbox_lineset[i].points = o3d.utility.Vector3dVector(vertices)
                    self.pred_bbox_lineset[i].lines = o3d.utility.Vector2iVector(lines)
                    
                    # 设置红色表示预测框
                    colors = np.array([[1, 0, 0] for _ in range(len(lines))])
                    self.pred_bbox_lineset[i].colors = o3d.utility.Vector3dVector(colors)
                    
                    self.vis.update_geometry(self.pred_bbox_lineset[i])
            
            # 更新真实边界框（可选）
            if gt_box_tensor is not None:
                if isinstance(gt_box_tensor, torch.Tensor):
                    gt_box_tensor = gt_box_tensor.cpu().numpy()
                if len(gt_box_tensor.shape) == 3 and gt_box_tensor.shape[0] == 1:
                    # 处理批处理维度
                    gt_box_tensor = gt_box_tensor[0]
                
                # 清除旧边界框
                for lineset in self.gt_bbox_lineset:
                    lineset.points = o3d.utility.Vector3dVector([])
                    lineset.lines = o3d.utility.Vector2iVector([])
                    lineset.colors = o3d.utility.Vector3dVector([])
                    self.vis.update_geometry(lineset)
                
                # 绘制新边界框
                for i, box in enumerate(gt_box_tensor):
                    if i >= len(self.gt_bbox_lineset):
                        break
                        
                    # 创建边界框顶点和线
                    vertices, lines = self._create_bbox_vertices_and_lines(box)
                    self.gt_bbox_lineset[i].points = o3d.utility.Vector3dVector(vertices)
                    self.gt_bbox_lineset[i].lines = o3d.utility.Vector2iVector(lines)
                    
                    # 设置绿色表示真实框
                    colors = np.array([[0, 1, 0] for _ in range(len(lines))])
                    self.gt_bbox_lineset[i].colors = o3d.utility.Vector3dVector(colors)
                    
                    self.vis.update_geometry(self.gt_bbox_lineset[i])
            
            # 更新渲染
            self.vis.poll_events()
            self.vis.update_renderer()
        
        except Exception as e:
            print(f"可视化V2X融合结果失败: {e}")
            import traceback
            traceback.print_exc()