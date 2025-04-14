import time
import os
import cv2
import open3d as o3d
import numpy as np
import threading

from opencda.core.common.misc import get_speed
from opencda.core.sensing.perception import sensor_transformation as st
from opencda.scenario_testing.utils.yaml_utils import save_yaml
from opencda.core.common.data_dumper import DataDumper as BaseDataDumper

# 全局共享数据存储
class SharedDataStorage:
    """全局数据共享存储类，用于实时存储和共享车辆感知数据"""
    _instance = None
    _lock = threading.Lock()
    
    def __new__(cls):
        with cls._lock:
            if cls._instance is None:
                cls._instance = super(SharedDataStorage, cls).__new__(cls)
                cls._instance.data = {}  # {vehicle_id: {frame: data_dict}}
                cls._instance.result = {} # {frame: objects}
                cls._instance.latest_frames = {}  # 存储每个车辆最新的帧号
                cls._instance.max_queue_size = 100
                cls._instance.last_cleanup_time = time.time()
                cls._instance.cleanup_interval = 2
            return cls._instance
    
    def add_data(self, vehicle_id, data_dict):
        """添加数据到存储中"""
        with self._lock:
            if vehicle_id not in self.data:
                self.data[vehicle_id] = {}
            
            # 使用帧号作为索引
            frame = data_dict['frame']
            self.data[vehicle_id][frame] = data_dict
            self.latest_frames[vehicle_id] = frame
            
            # 只保留最近的几个帧数据
            self._cleanup_vehicle_data(vehicle_id)
            
    def add_result(self, frame, objects):
        """添加数据到存储中"""
        with self._lock:
            self.result[frame] = objects
    
    def get_result(self, frame):
        """获取数据"""
        with self._lock:
            if frame not in self.result:
                return None
            return self.result[frame]
    
    def get_latest_data(self, vehicle_id):
        """获取特定车辆的最新数据"""
        with self._lock:
            if vehicle_id not in self.data or vehicle_id not in self.latest_frames:
                return None
            
            latest_frame = self.latest_frames[vehicle_id]
            return self.data[vehicle_id][latest_frame]
    
    def get_synchronized_data_by_frame(self, reference_frame, max_frame_diff=1):
        """
        获取与参考帧号同步的所有车辆数据
        
        Parameters
        ----------
        reference_frame : int
            参考帧号
        max_frame_diff : int
            允许的最大帧号差异
            
        Returns
        -------
        result : dict
            同步的数据字典 {vehicle_id: data_dict}
        """
        result = {}
        with self._lock:
            for vehicle_id in self.data:
                best_frame = None
                min_diff = float('inf')
                
                for frame in self.data[vehicle_id]:
                    diff = abs(frame - reference_frame)
                    if diff < min_diff and diff <= max_frame_diff:
                        min_diff = diff
                        best_frame = frame
                
                if best_frame is not None:
                    result[vehicle_id] = self.data[vehicle_id][best_frame]
        
        return result
    
    def _cleanup_vehicle_data(self, vehicle_id):
        """清理指定车辆的旧数据"""
        if vehicle_id in self.data:
            frames = sorted(self.data[vehicle_id].keys())
            if len(frames) > self.max_queue_size:
                # 删除最旧的数据
                for old_frame in frames[:-self.max_queue_size]:
                    del self.data[vehicle_id][old_frame]
    
    def get_all_vehicle_ids(self):
        """获取所有车辆ID"""
        with self._lock:
            return list(self.data.keys())


class V2XDataDumper(object):
    """
    Data dumper class to save data in local disk and shared memory.

    Parameters
    ----------
    perception_manager : opencda object
        The perception manager contains rgb camera data and lidar data.

    vehicle_id : int
        The carla.Vehicle id.


    Attributes
    ----------
    rgb_camera : list
        A list of opencda.CameraSensor that containing all rgb sensor data
        of the managed vehicle.

    lidar ; opencda object
        The lidar manager from perception manager.

    save_parent_folder : str
        The parent folder to save all data related to a specific vehicle.

    count : int
        Used to count how many steps have been executed. We dump data
        every 10 steps.
        
    shared_storage : SharedDataStorage
        共享数据存储对象，用于保存实时感知数据
    """

    def __init__(self,
                 perception_manager,
                 id,
                 save_time):


        if perception_manager.rgb_camera:
            self.rgb_camera = perception_manager.rgb_camera
        else: self.rgb_camera = []
        if perception_manager.lidar:
            self.lidar = perception_manager.lidar
        else: self.lidar = None

        self.id = id

        self.save_parent_folder = \
            os.path.join(os.getcwd(),
                         'data_dumping',
                         save_time,
                         str(self.id))

        if not os.path.exists(self.save_parent_folder):
            os.makedirs(self.save_parent_folder)

        self.count = 0
        
        # 初始化共享存储
        self.shared_storage = SharedDataStorage()

    def run_step(self, perception_manager, localization_manager, behavior_agent):
        """实时数据采集和共享"""
        self.count += 1
        # we ignore the first 60 steps
        if self.count < 60:
            return

        # 10hz
        if self.count % 2 != 0:
            return

        # 收集RGB图像和点云数据
        if self.rgb_camera: camera_data = self.collect_camera_data()
        else: camera_data = None
        if self.lidar: 
            lidar_data = self.collect_lidar_data()
            self.frame = lidar_data['frame']
        else: return
        yaml_data = self.collect_yaml_data(perception_manager, localization_manager, behavior_agent)

                    
        # 立即发布到共享存储，不用等待
        self.shared_storage.add_data(
            self.id, 
            {
                'lidar_data': lidar_data,
                'yaml_data': yaml_data,
                'frame': lidar_data['frame'],
                'camera_data': camera_data
            }
        )
        
        self.save_yaml_file(yaml_data, self.frame)
        self.save_lidar_points(lidar_data)
        self.save_rgb_image(self.frame, camera_data=camera_data)
        
        # 可选：仍然保存到磁盘，但作为低优先级任务
        # if self.save_to_disk and self.count % 5 == 0:  # 每5步保存一次
        #     threading.Thread(target=self.save_lidar_points, args=(lidar_data,)).start()
        #     threading.Thread(target=self.save_yaml_file, args=(yaml_data, self.count)).start()

    def collect_camera_data(self):
        """收集摄像头数据"""
        camera_data = []
        
        for (i, camera) in enumerate(self.rgb_camera):
            frame = camera.frame
            image = camera.image.copy()  # 复制图像数据
            
            camera_info = {
                'frame': frame,
                'image': image,
                'camera_id': i
            }
            camera_data.append(camera_info)
            
        return camera_data
    
    def collect_lidar_data(self):
        """收集激光雷达数据"""
        point_cloud = self.lidar.data.copy()  # 复制点云数据，避免引用问题
        frame = self.lidar.frame
        
        return {
            'points': point_cloud,
            'frame': frame
        }
    
    def collect_yaml_data(self, perception_manager, localization_manager, behavior_agent):
        """收集yaml数据"""
        dump_yml = {}
        vehicle_dict = {}

        # 收集障碍物车辆信息
        objects = perception_manager.objects
        vehicle_list = objects['vehicles'] if 'vehicles' in objects else []

        for veh in vehicle_list:
            veh_carla_id = getattr(veh, 'carla_id', -1)  # 使用getattr安全地获取属性
            veh_pos = veh.get_transform() if hasattr(veh, 'get_transform') else None
            veh_bbx = getattr(veh, 'bounding_box', None)
            veh_speed = get_speed(veh) if hasattr(veh, 'get_velocity') else 0
            
            # 安全地获取类型ID和颜色
            veh_type_id = getattr(veh, 'type_id', None)
            veh_color = getattr(veh, 'color', None)
            
            vehicle_dict.update({veh_carla_id: {
                'bp_id': veh_type_id if veh_type_id is not None else 'unknown_type',
                'color': veh_color if veh_color else 'unknown_color',
                "location": [veh_pos.location.x,
                             veh_pos.location.y,
                             veh_pos.location.z] if veh_pos and hasattr(veh_pos, 'location') else [0, 0, 0],
                "center": [veh_bbx.location.x,
                           veh_bbx.location.y,
                           veh_bbx.location.z] if veh_bbx and hasattr(veh_bbx, 'location') else [0, 0, 0],
                "angle": [veh_pos.rotation.roll,
                          veh_pos.rotation.yaw,
                          veh_pos.rotation.pitch] if veh_pos and hasattr(veh_pos, 'rotation') else [0, 0, 0],
                "extent": [veh_bbx.extent.x,
                           veh_bbx.extent.y,
                           veh_bbx.extent.z] if veh_bbx and hasattr(veh_bbx, 'extent') else [0, 0, 0],
                "speed": veh_speed if veh_speed is not None else 0,
            }})

        dump_yml.update({'vehicles': vehicle_dict})

        # 收集自车位姿和速度
        predicted_ego_pos = localization_manager.get_ego_pos()
        true_ego_pos = localization_manager.vehicle.get_transform() \
            if hasattr(localization_manager, 'vehicle') \
            else localization_manager.true_ego_pos

        dump_yml.update({'predicted_ego_pos': [
            predicted_ego_pos.location.x,
            predicted_ego_pos.location.y,
            predicted_ego_pos.location.z,
            predicted_ego_pos.rotation.roll,
            predicted_ego_pos.rotation.yaw,
            predicted_ego_pos.rotation.pitch]})
        dump_yml.update({'true_ego_pos': [
            true_ego_pos.location.x,
            true_ego_pos.location.y,
            true_ego_pos.location.z,
            true_ego_pos.rotation.roll,
            true_ego_pos.rotation.yaw,
            true_ego_pos.rotation.pitch]})
        dump_yml.update({'ego_speed':
                        float(localization_manager.get_ego_spd())})

        # 收集激光雷达传感器位姿
        lidar_transformation = self.lidar.sensor.get_transform()
        dump_yml.update({'lidar_pose': [
            lidar_transformation.location.x,
            lidar_transformation.location.y,
            lidar_transformation.location.z,
            lidar_transformation.rotation.roll,
            lidar_transformation.rotation.yaw,
            lidar_transformation.rotation.pitch]})

        # 收集相机传感器信息
        for (i, camera) in enumerate(self.rgb_camera):
            camera_param = {}
            camera_transformation = camera.sensor.get_transform()
            camera_param.update({'cords': [
                camera_transformation.location.x,
                camera_transformation.location.y,
                camera_transformation.location.z,
                camera_transformation.rotation.roll,
                camera_transformation.rotation.yaw,
                camera_transformation.rotation.pitch
            ]})

            # 内参矩阵
            camera_intrinsic = st.get_camera_intrinsic(camera.sensor)
            camera_intrinsic = self.matrix2list(camera_intrinsic)
            camera_param.update({'intrinsic': camera_intrinsic})

            # 外参矩阵
            lidar2world = \
                st.x_to_world_transformation(self.lidar.sensor.get_transform())
            camera2world = \
                st.x_to_world_transformation(camera.sensor.get_transform())

            world2camera = np.linalg.inv(camera2world)
            lidar2camera = np.dot(world2camera, lidar2world)
            lidar2camera = self.matrix2list(lidar2camera)
            camera_param.update({'extrinsic': lidar2camera})
            dump_yml.update({'camera%d' % i: camera_param})

        dump_yml.update({'RSU': True})
        
        # 规划轨迹
        if behavior_agent is not None:
            trajectory_deque = \
                behavior_agent.get_local_planner().get_trajectory()
            trajectory_list = []

            for i in range(len(trajectory_deque)):
                tmp_buffer = trajectory_deque.popleft()
                x = tmp_buffer[0].location.x
                y = tmp_buffer[0].location.y
                spd = tmp_buffer[1]

                trajectory_list.append([x, y, spd])

            dump_yml.update({'plan_trajectory': trajectory_list})
            dump_yml.update({'RSU': False})
            
        return dump_yml

    def save_rgb_image(self, count, camera_data=None):
        """
        Save camera rgb images to disk.
        
        Parameters
        ----------
        count : int
            Frame count
        camera_data : list, optional
            预先收集的相机数据，如果为None则实时获取
        """
        if camera_data is None:
            # 使用之前的方式保存
            for (i, camera) in enumerate(self.rgb_camera):
                frame = camera.frame
                image = camera.image

                image_name = '%06d' % count + '_' + 'camera%d' % i + '.png'

                cv2.imwrite(os.path.join(self.save_parent_folder, image_name),
                            image)
        else:
            # 使用预先收集的数据保存
            for camera_info in camera_data:
                i = camera_info['camera_id']
                image = camera_info['image']
                
                image_name = '%06d' % count + '_' + 'camera%d' % i + '.png'
                cv2.imwrite(os.path.join(self.save_parent_folder, image_name), image)

    def save_lidar_points(self, lidar_data=None):
        """
        Save 3D lidar points to disk.
        
        Parameters
        ----------
        lidar_data : dict, optional
            预先收集的激光雷达数据，如果为None则实时获取
        """
        if lidar_data is None:
            # 使用之前的方式保存
            point_cloud = self.lidar.data
            frame = self.lidar.frame
        else:
            # 使用预先收集的数据保存
            point_cloud = lidar_data['points']
            frame = lidar_data['frame']

        point_xyz = point_cloud[:, :-1]
        point_intensity = point_cloud[:, -1]
        point_intensity = np.c_[
            point_intensity,
            np.zeros_like(point_intensity),
            np.zeros_like(point_intensity)
        ]

        o3d_pcd = o3d.geometry.PointCloud()
        o3d_pcd.points = o3d.utility.Vector3dVector(point_xyz)
        o3d_pcd.colors = o3d.utility.Vector3dVector(point_intensity)

        # write to pcd file
        pcd_name = '%06d' % self.frame + '.pcd'
        o3d.io.write_point_cloud(os.path.join(self.save_parent_folder,
                                             pcd_name),
                                 pointcloud=o3d_pcd,
                                 write_ascii=True)

    def save_yaml_file(self, yaml_data, count):
        """
        Save yaml data to disk.
        
        Parameters
        ----------
        yaml_data : dict
            要保存的yaml数据
        count : int
            Frame count
        """
        yml_name = '%06d' % count + '.yaml'
        save_path = os.path.join(self.save_parent_folder, yml_name)
        save_yaml(yaml_data, save_path)

    @staticmethod
    def matrix2list(matrix):
        """
        To generate readable yaml file, we need to convert the matrix
        to list format.

        Parameters
        ----------
        matrix : np.ndarray
            The extrinsic/intrinsic matrix.

        Returns
        -------
        matrix_list : list
            The matrix represents in list format.
        """
        assert len(matrix.shape) == 2
        return matrix.tolist()