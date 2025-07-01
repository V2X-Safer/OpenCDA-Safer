# OpenCDA 环境搭建与使用说明

## 测试环境

| CPU      | 13th Gen Intel(R) Core(TM) i9-13900K                         |
| -------- | :----------------------------------------------------------- |
| GPU      | NVIDIA GeForce RTX 4090 (×1)                                 |
| RAM      | 64GB / DDR5 4800MHz (Corsair ×2)                             |
| 硬盘空间 | 1TB NVMe SSD (Samsung 980 PRO) + 500GB NVMe SSD (ZHITAI Ti600) + 3.6TB SATA HDD (ST4000NM0035) |

## 安装步骤

### 1. Conda 环境准备

```shell
cd OpenCDA
conda env create -f v2xfuzz_opencda_environment.yml
conda activate v2xfuzz
which pip # 验证是否成功
pip install -r v2xfuzz_opencda_requirements.txt
```

### 2. 安装 CARLA PythonAPI
- 源码目录已附带 Carla 0.9.12/0.9.10，分别位于 `/carla_0_9_12` 与 `/V2Xverse/carla`
- 使用前需加载环境变量：

```shell
source v2xfuzz_opencda_env.sh
export PYTHONPATH=$PYTHONPATH:/path/to/CARLA/PythonAPI/carla/dist/carla-<version>-py3.7-linux-x86_64.egg
```

### 3. 安装 OpenCOOD（可选，推荐集成）

```shell
cd opencood
pip install -r requirements.txt
```
- 按照 [OpenCOOD installation](https://opencood.readthedocs.io/en/latest/md_files/installation.html) 安装，spconv 建议2.0版本。

### 4. 运行 OpenCDA/Fuzz 测试

1. 配置测试场景：在 `src/test_yaml/` 目录下编写 YAML 文件，定义世界参数、交通流、车队等。
2. 启动 CARLA 仿真服务器，并设置 DISPLAY 变量（如远程需用物理机 DISPLAY）：
```shell
export DISPLAY=:1
./CarlaUE4.sh &
```
3. 运行 Fuzz 测试：
```shell
cd src
python main.py
```

---

## 参考文档
- [OpenCDA 官方文档](https://opencda-documentation.readthedocs.io/en/latest/md_files/installation.html)
- [OpenCOOD 官方文档](https://opencood.readthedocs.io/en/latest/md_files/installation.html)

如需自定义变异策略或集成新场景，请参考源码中的 `Scenario_`、`operation.py`、`ml_manager.py` 等文件。
