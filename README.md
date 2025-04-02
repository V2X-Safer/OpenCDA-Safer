# 车队安全性能测试Fuzz系统

## 项目概述

这是一个基于OpenCDA平台开发的Fuzz测试系统，专门用于测试车队（Platoon）的安全性能，包括多车感知、多车规划等关键模块。本系统通过模拟各种复杂场景、变异测试条件以及系统扰动，全面评估车队行驶的稳定性与安全性。

## 项目结构
总体结构
```
OpenCDA/
├── opencda/                   # OpenCDA核心代码
├── src/                       # 源码目录
├── docs/                      # OpenCDA文档
└── TODO.md                    # 待办事项清单
```

Fuzzer结构
``` FUZZ
src
├── __init__.py
├── log                         # 日志
│   ├── param                   # 用以保存每次运行的参数
│   └── view  
├── main.py                     # 主函数 Fuzzer的逻辑
├── operation.py                # 算子操作 变异使用
├── opt.py                      # 配置选项
├── oracle_manager.py           # oracle计算
├── Scenario_.py                # 单次场景的运行
├── test_yaml                   # 测试场景 (种子场景位于 opencda/scenario_testing/config_yaml)
│   ├── default.yaml
│   ├── openscenario_carla.yaml
│   ├── platoon_joining_2lanefree_carla.yaml
│   ├── platoon_joining_2lanefree_cosim.yaml
│   ├── platoon_joining_town06_carla.yaml
|   ...
└── utils_.py     
```


## 核心功能

### 1. Oracle评估机制
系统通过以下多种维度评估车队的安全性能：
- **车间距分析**：监控和评估车队内部车辆间距的合理性
- **碰撞检测**：识别和记录车辆间的碰撞事件
- **驾驶质量评估**：检测急刹车、急加速、急转弯和长时间停车等异常行为
- **交通规则遵守度**：监控对交通信号和车道线的遵守情况
- **任务完成度**：评估车队合流成功率和目标点到达情况

### 2. 操作变异功能（Operator）
系统可以变异以下环境与车辆参数：
- **天气条件**：模拟各种天气状况（雨、云、风、雾、潮湿、水坑等）和太阳位置
- **交通流密度**：调整周围车辆密度以测试不同交通负荷下的表现
- **智能体操作**：变异车辆和行人的类型、位置和数量
- **通信扰动**：模拟V2X通信中的各种噪声和干扰情况

### 3. 多进程测试
每次运行场景则spawn一个进程，用以防止`Carla`的bug

## 使用方法

### 环境准备
1. 确保已安装CARLA模拟器
2. 安装OpenCDA依赖
```bash
cd OpenCDA
pip install -e .
```

### 运行测试
1. 启动CARLA服务器
注意：如果通过ssh连接，启动`Carla`时记得设置DISPLAY变量，此处使用使用物理机的DISPLAY变量，查看运行情况请连接todesk查看
```bash
export DISPLAY=:1 # 设置DISPLAY变量
cd /home/test/V2X/OpenCDA/carla_0_9_12
./CarlaUE4.sh &
```

2. 运行Fuzz测试
```bash
cd OpenCDA/src
conda activate opencda
python main.py
```
参数修改请前往opt.py修改，若只需要执行单个场景测试，可运行
```bash
python Scenario_.py
```

### 配置测试场景s
在配置文件中可以设定以下参数：
- 测试地图与场景
- 车辆参数配置
- 变异策略与权重
- 评估指标阈值

### 结果分析
测试完成后，系统会生成详细的测试报告，包括：
- 各评估指标的得分统计
- 失败场景的详细信息与回放数据
- 系统性能瓶颈分析

## 开发计划

参考项目的TODO.md文件，当前开发重点包括：
- 完善变异策略权重设计
- 修复已知的多进程队列通信问题
- 添加更多场景模板与测试案例
---

*注意：使用前请确保您已理解CARLA和OpenCDA的基本操作原理。不当的配置可能导致测试结果不准确。*