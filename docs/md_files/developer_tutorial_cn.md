### 类设计

本节将深入探讨OpenCDA框架中几个核心类的实现细节。建议初学者先阅读[OpenCDA逻辑流程](tutorial.md)以理解整体仿真流程。本教程将重点解析类的设计逻辑，并阐述各模块的核心算法。我们以测试案例`platoon_joining_2lanefree_carla.py`为例进行讲解，为便于理解已简化非核心代码，完整代码请参考[代码库](https://github.com/ucla-mobility/OpenCDA)，协同架构细节请参阅[论文](https://arxiv.org/abs/2107.06260)。

<strong>注意：本教程假设仅使用CARLA单机仿真。</strong>

### 工作流程

OpenCDA工作流程概览：

* 编写YAML文件定义仿真配置
* 通过`load_yaml`加载配置到字典`scenario_params`
* 创建`CavWorld`对象存储CAV注册信息及共享模型
* `ScenarioManager`根据配置初始化`carla.TrafficManager`并加载地图
* 使用`create_vehicle_manager`生成CAV车辆管理器（`VehicleManager`）
* 通过`create_platoon_manager`生成车队管理器（`PlatooningManager`）
* `create_traffic_carla`生成背景交通流
* 进入仿真循环，通过`scenario_manager.tick()`推进仿真
* 各CAV更新状态并执行控制指令，处理车队合并逻辑

```python
from opencda.scenario_testing.utils.customized_map_api import customized_map_helper
def run_scenario(opt, config_yaml):
    # 加载场景配置
    scenario_params = load_yaml(config_yaml)
    xodr_path = "path/to/customized_map.xodr"
    # 创建CAV世界
    cav_world = CavWorld(opt.apply_ml)
    # 初始化场景管理器
    scenario_manager = sim_api.ScenarioManager(scenario_params,opt.apply_ml,xodr_path=xodr_path,cav_world=cav_world)
    # 生成单车CAV列表
    single_cav_list = scenario_manager.create_vehicle_manager(['platooning'],map_helper=customized_map_helper)
    # 生成车队
    platoon_list = scenario_manager.create_platoon_manager(data_dump=False)
    # 生成背景交通
    traffic_manager, bg_veh_list = scenario_manager.create_traffic_carla()
    # 主循环
    while True:
        scenario_manager.tick()
        # 更新车队状态
        for platoon in platoon_list:
            platoon.update_information()
            platoon.run_step()
        # 更新单车CAV
        for i, single_cav in enumerate(single_cav_list):
            if single_cav.v2x_manager.in_platoon():
                single_cav_list.pop(i)
            else:
                single_cav.update_info()
                control = single_cav.run_step()
                single_cav.vehicle.apply_control(control)
```

### CavWorld

`CavWorld`作为信息中枢，主要功能包括：

* 存储CAV车辆ID集合`vehicle_id_set`
* 维护车辆管理器字典`_vehicle_manager_dict`
* 管理车队字典`_platooning_dict`
* 加载共享机器学习模型（通过`ml_manager`）

```python
class CavWorld(object):
    def __init__(self, apply_ml=False):
        self.vehicle_id_set = set()  # carla.Vehicle ID集合
        self._vehicle_manager_dict = {}  # (vid, VehicleManager)映射
        self._platooning_dict = {}  # (pmid, PlatooningManager)映射
        self.ml_manager = None  # 机器学习模型管理器
        if apply_ml:
            self.ml_manager = function_to_load_ML_model()
```

### ScenarioManager

场景管理核心类，主要职责：

* 初始化CARLA客户端与地图
* 配置仿真参数（同步模式、时间步长等）
* 提供三大生成方法：
  * `create_vehicle_manager`：生成单车CAV
  * `create_platoon_manager`：生成车队
  * `create_traffic_carla`：生成背景交通

```python
class ScenarioManager:
    def __init__(self, scenario_params, apply_ml, xodr_path=None):
        # 初始化CARLA客户端
        self.client = carla.Client('localhost', simulation_config['client_port'])
        # 加载定制地图
        self.world = load_customized_world(xodr_path, self.client)
        # 配置仿真参数
        setting = self.world.get_settings()
        setting.synchronous_mode = True
        setting.fixed_delta_seconds = simulation_config['fixed_delta_seconds']
        self.world.apply_settings(new_settings)
```

#### create_vehicle_manager 实现解析

```python
def create_vehicle_manager(self, application, map_helper=None):
    cav_vehicle_bp = self.world.get_blueprint_library().find('vehicle.lincoln.mkz2017')
    single_cav_list = []
    for cav_config in scenario_params['scenario']['single_cav_list']:
        # 生成车辆实例
        spawn_transform = get_spawn_transform(cav_config)
        vehicle = self.world.spawn_actor(cav_vehicle_bp, spawn_transform)
        # 创建车辆管理器
        vehicle_manager = VehicleManager(vehicle, cav_config, application,...)
        # 设置目的地
        destination = carla.Location(x=cav_config['destination'][0],...)
        vehicle_manager.set_destination(..., destination)
        single_cav_list.append(vehicle_manager)
    return single_cav_list
```

#### create_platoon_manager 实现解析

```python
def create_platoon_manager(self):
    platoon_list = []
    for platoon in scenario_params['scenario']['platoon_list']:
        platoon_manager = PlatooningManager(...)
        # 生成车队成员
        for member in platoon['members']:
            vehicle = self.world.spawn_actor(...)
            vehicle_manager = VehicleManager(...)
            # 设置头车/跟随车
            if j == 0: platoon_manager.set_lead(vehicle_manager)
            else: platoon_manager.add_member(vehicle_manager)
        platoon_list.append(platoon_manager)
    return platoon_list
```

### VehicleManager

车辆管理核心类，集成以下模块：

* **V2X通信**：`V2XManager`
* **环境感知**：`PerceptionManager`
* **定位导航**：`LocalizationManager`
* **决策规划**：`BehaviorAgent`
* **运动控制**：`ControlManager`

```python
class VehicleManager(object):
    def __init__(self, vehicle, config_yaml, application, ...):
        # 初始化各功能模块
        self.v2x_manager = V2XManager(...)
        self.localizer = LocalizationManager(...)
        self.perception_manager = PerceptionManager(...)
        self.agent = BehaviorAgent(...)
        self.controller = ControlManager(...)
```

关键方法说明：

| 方法名称       | 功能描述                                                                 |
|----------------|--------------------------------------------------------------------------|
| `update_info`  | 更新定位/感知信息并同步至各模块                                         |
| `run_step`     | 执行决策规划→生成控制指令                                               |
| `set_destination` | 设置目标位置并初始化全局路径规划                                       |

### PerceptionManager

环境感知模块，主要特性：

* 支持多传感器融合（摄像头、激光雷达）
* 提供两种感知模式：
  - **真实模式**：使用YOLOv5进行目标检测
  - **简化模式**：直接获取服务器数据

```python
class PerceptionManager:
    def __init__(self, vehicle, config_yaml, ...):
        # 传感器配置
        self.rgb_camera = [CameraSensor(...) for _ in range(3)]
        self.lidar = LidarSensor(...)
        # 感知结果存储
        self.objects = {'vehicles': [], 'traffic_lights': []}
```

核心方法`detect`工作流程：
1. 获取原始感知数据
2. 根据激活状态选择检测方式
3. 过滤白名单车辆
4. 返回结构化环境信息

### LocalizationManager

定位导航模块核心技术：

* 多传感器数据融合（GNSS+IMU）
* 卡尔曼滤波实现状态估计
* 噪声注入模拟真实传感器误差

```python
class LocalizationManager(object):
    def __init__(self, vehicle, config_yaml, ...):
        # 传感器初始化
        self.gnss = GnssSensor(...)
        self.imu = ImuSensor(...)
        # 状态估计
        self.kf = KalmanFilter(config_yaml['dt'])
        # 历史轨迹存储
        self._ego_pos_history = deque(maxlen=100)
```

### BehaviorAgent

决策规划核心组件，功能架构：

```python
class BehaviorAgent(object):
    def __init__(self, ...):
        # 双级规划体系
        self._global_planner = GlobalRoutePlanner(...)  # 全局路径规划
        self._local_planner = LocalPlanner(...)         # 局部轨迹规划
        # 交通规则处理
        self.light_state = "Red"  # 信号灯状态跟踪
        # 安全校验
        self._collision_check = CollisionChecker(...)   # 碰撞检测
```

典型决策场景处理逻辑：
1. **信号灯控制**：红灯停/绿灯行逻辑，处理路口停滞问题
2. **车道变更**：基于路网拓扑和交通流状态决策
3. **跟车模式**：保持安全车距的PID控制
4. **紧急制动**：基于TTC（碰撞时间）的制动策略

### V2XManager

车联网通信模块设计要点：

```python
class V2XManager(object):
    def __init__(self, ...):
        # 通信参数配置
        self.communication_range = 200  # 通信半径（米）
        self.lag = 3                   # 通信延迟（帧）
        # 数据缓冲区
        self.ego_pos = deque(maxlen=100)  # 历史位置信息
        # 插件系统
        self.platooning_plugin = PlatooningPlugin(...)  # 车队协同插件
```

核心方法`update_info`工作流程：
1. 更新本车状态信息
2. 搜索通信范围内邻近车辆
3. 同步信息至各功能插件
4. 添加噪声/延迟模拟真实通信环境

通过这种模块化设计，OpenCDA实现了灵活可扩展的协同驾驶仿真框架，为自动驾驶算法研发提供了完整的测试验证平台。