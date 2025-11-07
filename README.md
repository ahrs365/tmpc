# MPC Planner - 纯 C++ 实现

基于模型预测控制（MPC）的移动机器人路径规划器，采用 T-MPC++ 算法，支持动态障碍物避障和拓扑引导路径规划。本项目为纯 C++ 实现，无 ROS 依赖，可独立运行。

![MPC Planner Simulation](docs/jackalsimulator.gif)

---

## 📋 目录

- [项目简介](#项目简介)
- [核心特性](#核心特性)
- [算法原理](#算法原理)
- [项目结构](#项目结构)
- [系统要求](#系统要求)
- [安装与编译](#安装与编译)
- [快速开始](#快速开始)
- [使用指南](#使用指南)
- [场景配置](#场景配置)
- [可视化界面](#可视化界面)
- [配置参数](#配置参数)
- [常见问题](#常见问题)
- [参考文献](#参考文献)

---

## 🎯 项目简介

MPC Planner 是一个高性能的移动机器人路径规划系统，采用模型预测控制（Model Predictive Control, MPC）算法实现实时路径规划和障碍物避障。

### 核心算法：T-MPC++

T-MPC++（Topology-guided Model Predictive Control）是一种结合拓扑引导和并行优化的先进 MPC 算法：

1. **拓扑引导**：使用引导路径规划器（Guidance Planner）生成多条不同拓扑类别的候选路径
2. **并行优化**：为每条引导路径并行运行独立的 MPC 求解器
3. **最优选择**：从所有成功的候选轨迹中选择代价最小的作为最终控制输出

### 应用场景

- 移动机器人导航
- 自动驾驶车辆路径规划
- 无人机避障飞行
- 仓储物流机器人

---

## ✨ 核心特性

### 算法特性
- ✅ **T-MPC++ 算法**：拓扑引导 + 并行 MPC 优化
- ✅ **动态障碍物避障**：支持动态圆形和矩形障碍物
- ✅ **实时性能**：基于 ACADOS 的高效 QP 求解器
- ✅ **轮廓跟踪**：Contouring 控制策略，优化路径跟踪性能
- ✅ **多拓扑路径**：自动生成和评估多条不同拓扑的候选路径

### 工程特性
- ✅ **纯 C++ 实现**：无 ROS 依赖，可独立运行
- ✅ **模块化设计**：清晰的代码结构，易于扩展
- ✅ **可视化界面**：基于 matplotlib-cpp 的实时可视化
- ✅ **场景配置**：支持 JSON 格式的场景文件
- ✅ **中文支持**：界面和文档支持中文显示

---

## 🧮 算法原理

### 1. 模型预测控制（MPC）

MPC 是一种基于优化的控制方法，在每个控制周期求解以下优化问题：

```
min  Σ (轨迹代价 + 控制代价)
s.t. 动力学约束
     障碍物避障约束
     控制输入约束
     状态约束
```

**优势**：
- 显式处理约束
- 预测未来状态
- 优化控制性能

### 2. T-MPC++ 算法流程

```
1. 引导路径规划
   └─> 使用 RRT* + 同伦类分类生成多条候选引导路径

2. 并行 MPC 优化
   ├─> 为每条引导路径启动独立的 MPC 求解器
   ├─> 每个求解器优化自己的轨迹
   └─> 同时运行一个无引导的 MPC 求解器（备选）

3. 轨迹选择
   ├─> 收集所有成功的候选轨迹
   ├─> 根据代价函数评估每条轨迹
   └─> 选择代价最小的轨迹作为输出

4. 控制执行
   └─> 执行选定轨迹的第一个控制输入
```

### 3. 动力学模型

支持二阶单轮车模型（Second-Order Unicycle Model）：

```
状态: [x, y, ψ, v, θ]
  x, y: 位置
  ψ: 航向角
  v: 线速度
  θ: 样条参数（用于轮廓跟踪）

控制输入: [a, ω]
  a: 线加速度
  ω: 角速度
```

### 4. 障碍物避障

- **静态障碍物**：圆形、多边形（自动近似为圆形）
- **动态障碍物**：圆形、矩形（使用外接圆近似）
- **避障约束**：使用椭圆约束或线性化约束

---

## 📁 项目结构

```
mpc_planner/
├── main.cpp                      # 主程序入口
├── build_run_sim.sh             # 编译和运行脚本
├── CMakeLists.txt               # 顶层 CMake 配置
│
├── scenarios/                    # 场景配置文件
│   ├── scenario_buttons.yaml   # 场景按钮配置
│   ├── 密集动态.json            # 场景文件示例
│   └── ...                      # 其他场景文件
│
├── mpc_planner_solver/          # MPC 求解器模块
│   ├── include/                 # 求解器头文件
│   ├── src/                     # 求解器实现
│   ├── Solver/                  # ACADOS 生成的求解器代码
│   └── config/                  # 求解器配置文件
│
├── guidance_planner/            # 引导路径规划器
│   ├── include/                 # 引导规划器头文件
│   ├── src/                     # RRT* + 同伦类分类实现
│   └── config/                  # 引导规划器配置
│
├── mpc_planner_modules/         # MPC 核心模块
│   ├── include/                 # 模块头文件
│   └── src/                     # Contouring、T-MPC++ 实现
│
├── mpc_planner_types/           # 数据类型定义
│   ├── include/                 # 类型头文件（State, Trajectory 等）
│   └── src/                     # 类型实现
│
├── mpc_planner_util/            # 工具库
│   ├── include/                 # 工具函数头文件
│   └── src/                     # 配置加载、日志等工具
│
├── ros_tools_no_ros/            # ROS 工具的无依赖实现
│   ├── include/                 # 时间、日志等工具
│   └── src/                     # 实现代码
│
├── mpc_planner_jackalsimulator/ # Jackal 仿真器配置
│   └── config/                  # 仿真器参数配置
│
├── DecompUtil/                  # 凸分解工具库
│   └── include/                 # 多边形分解、椭圆约束
│
├── solver_generator/            # 求解器生成工具
│   ├── generate_solver.py      # 求解器生成脚本
│   └── solver_model.py          # 动力学模型定义
│
├── third_party/                 # 第三方库
│   ├── yaml-cpp/               # YAML 解析库
│   ├── matplotlib-cpp/         # Python matplotlib 的 C++ 封装
│   ├── simple_json.hpp         # 轻量级 JSON 解析器
│   └── nlohmann_json.hpp       # JSON 库（备用）
│
├── docs/                        # 文档
│   ├── configuration_guide.md  # 配置指南
│   ├── visualization_controls.md # 可视化控制说明
│   └── ...                      # 其他文档
│
└── build/                       # 编译输出目录
    └── pure_cpp/               # 纯 C++ 版本可执行文件
        └── mpc_planner_main    # 主程序
```



### 各模块功能详细说明

#### 1. `mpc_planner_solver/` - MPC 求解器模块
**功能**：基于 ACADOS 的 MPC 求解器实现
- 封装 ACADOS QP 求解器接口
- 实现动力学模型（二阶单轮车模型）
- 处理约束条件（障碍物避障、控制限制等）
- 提供求解器配置和参数调整接口

**依赖**：ACADOS, Eigen3

**关键文件**：
- `Solver/`：ACADOS 自动生成的求解器代码
- `include/mpc_planner_solver/`：求解器接口头文件
- `config/`：求解器参数配置文件

#### 2. `guidance_planner/` - 引导路径规划器
**功能**：生成多条不同拓扑类别的引导路径
- RRT* 算法实现快速路径搜索
- 同伦类分类（Homotopy Class）识别不同拓扑路径
- 路径平滑和优化
- 为 T-MPC++ 提供引导路径

**依赖**：DecompUtil, Eigen3

**关键文件**：
- `include/guidance_planner/`：引导规划器接口
- `src/`：RRT*、同伦类分类实现
- `config/`：引导规划器参数配置

#### 3. `mpc_planner_modules/` - MPC 核心模块
**功能**：MPC 规划器的核心逻辑实现
- Contouring 控制器：优化路径跟踪性能
- T-MPC++ 算法：并行 MPC 优化和轨迹选择
- 障碍物预测和避障逻辑
- 代价函数计算和轨迹评估

**依赖**：mpc_planner_solver, guidance_planner

**关键文件**：
- `include/mpc_planner_modules/`：核心模块接口
- `src/`：Contouring、T-MPC++ 实现

#### 4. `mpc_planner_types/` - 数据类型定义
**功能**：定义项目中使用的所有数据结构
- `State`：机器人状态（位置、速度、航向等）
- `Trajectory`：轨迹数据结构
- `Obstacle`：障碍物数据结构
- `PlannerOutput`：规划器输出结构

**依赖**：Eigen3

**关键文件**：
- `include/mpc_planner_types/`：类型定义头文件
- `src/`：类型实现代码

#### 5. `mpc_planner_util/` - 工具库
**功能**：提供各种工具函数和辅助功能
- 配置文件加载（YAML 格式）
- 日志系统（LOG_INFO, LOG_WARN, LOG_ERROR）
- 数学工具函数
- 坐标变换工具

**依赖**：yaml-cpp, Eigen3

**关键文件**：
- `include/mpc_planner_util/`：工具函数头文件
- `src/`：工具函数实现

#### 6. `ros_tools_no_ros/` - ROS 工具的无依赖实现
**功能**：提供 ROS 常用工具的独立实现，无需 ROS 环境
- 时间管理（Time, Duration）
- 参数服务器（Parameter Server）
- 日志系统（ROS_INFO, ROS_WARN 等宏）
- 节点句柄（NodeHandle）模拟

**依赖**：无

**关键文件**：
- `include/ros_tools_no_ros/`：ROS 工具接口
- `src/`：独立实现代码

#### 7. `mpc_planner_jackalsimulator/` - Jackal 仿真器配置
**功能**：Jackal 机器人的仿真参数配置
- 机器人物理参数（尺寸、质量等）
- 控制参数（速度限制、加速度限制等）
- 传感器参数
- 仿真环境配置

**依赖**：无

**关键文件**：
- `config/`：仿真器 YAML 配置文件

#### 8. `DecompUtil/` - 凸分解工具库
**功能**：多边形凸分解和椭圆约束生成
- 多边形凸分解算法
- 椭圆障碍物约束生成
- 几何工具函数

**依赖**：Eigen3

**关键文件**：
- `include/decomp_util/`：凸分解工具接口

#### 9. `solver_generator/` - 求解器生成工具
**功能**：自动生成 ACADOS 求解器代码
- 动力学模型定义（Python）
- 约束条件定义
- 代价函数定义
- 自动生成 C 代码求解器

**依赖**：Python3, ACADOS Python 接口

**关键文件**：
- `generate_solver.py`：求解器生成主脚本
- `solver_model.py`：动力学模型定义
- `control_modules.py`：控制模块定义

#### 10. `third_party/` - 第三方库
**功能**：项目依赖的第三方库
- `yaml-cpp/`：YAML 文件解析库
- `matplotlib-cpp/`：Python matplotlib 的 C++ 封装，用于可视化
- `simple_json.hpp`：轻量级 JSON 解析器（单头文件）
- `nlohmann_json.hpp`：功能完整的 JSON 库（备用）

#### 11. `scenarios/` - 场景配置文件
**功能**：存储仿真场景的 JSON 配置文件
- 起点和终点定义
- 静态障碍物配置
- 动态障碍物配置
- 场景按钮映射配置

**文件格式**：JSON

**关键文件**：
- `scenario_buttons.yaml`：场景按钮配置
- `*.json`：各种场景配置文件

#### 12. `docs/` - 文档
**功能**：项目文档和说明
- 配置指南
- 可视化控制说明
- 模型适配指南
- 算法实现说明

---

## 💻 系统要求

### 操作系统
- Ubuntu 20.04 / 22.04 / 24.04
- 其他 Linux 发行版（需要相应调整依赖安装）

### 编译器
- GCC 9.0+ 或 Clang 10.0+
- 支持 C++17 标准

### 依赖库

#### 必需依赖
- **CMake** >= 3.8
- **Eigen3** >= 3.3
- **Python3** >= 3.8（用于可视化）
- **ACADOS**（MPC 求解器）
- **BLAS/LAPACK**（线性代数库）

#### Python 依赖
- `matplotlib` >= 3.0（可视化）
- `numpy`（数值计算）

#### 可选依赖
- **中文字体**：`fonts-wqy-microhei` 或 `fonts-wqy-zenhei`（用于界面中文显示）

---

## 🔧 安装与编译

### 1. 安装系统依赖

```bash
# 更新包列表
sudo apt-get update

# 安装编译工具
sudo apt-get install -y build-essential cmake git

# 安装 Eigen3
sudo apt-get install -y libeigen3-dev

# 安装 BLAS/LAPACK
sudo apt-get install -y libblas-dev liblapack-dev

# 安装 Python 和 matplotlib
sudo apt-get install -y python3 python3-pip python3-dev
pip3 install matplotlib numpy

# 安装中文字体（可选，用于界面中文显示）
sudo apt-get install -y fonts-wqy-microhei fonts-wqy-zenhei

# 清除 matplotlib 字体缓存（安装中文字体后需要）
rm -rf ~/.cache/matplotlib
```

### 2. 安装 ACADOS

```bash
# 克隆 ACADOS 仓库
cd ~
git clone https://github.com/acados/acados.git
cd acados
git submodule update --init --recursive

# 编译 ACADOS
mkdir -p build
cd build
cmake -DACADOS_WITH_QPOASES=ON ..
make -j$(nproc)
sudo make install

# 设置环境变量
echo 'export ACADOS_SOURCE_DIR=~/acados' >> ~/.bashrc
echo 'export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/acados/lib' >> ~/.bashrc
source ~/.bashrc
```

### 3. 克隆项目

```bash
cd ~/workspace
git clone <repository_url> mpc_planner
cd mpc_planner

# 初始化子模块
git submodule update --init --recursive
```

### 4. 编译项目

```bash
# 方法 1：使用便捷脚本（推荐）
./build_run_sim.sh

# 方法 2：手动编译
mkdir -p build/pure_cpp
cd build/pure_cpp
cmake ../..
make -j$(nproc)
```

编译成功后，可执行文件位于 `build/pure_cpp/mpc_planner_main`。

---

## 🚀 快速开始

### 运行默认场景

```bash
# 使用便捷脚本
./build_run_sim.sh

# 或手动运行
cd build/pure_cpp
./mpc_planner_main ../../mpc_planner_jackalsimulator/config
```

### 运行指定场景

```bash
# 使用场景名称
./build_run_sim.sh 密集动态

# 或使用完整路径
cd build/pure_cpp
./mpc_planner_main ../../mpc_planner_jackalsimulator/config ../../scenarios/密集动态.json
```

### 可用场景列表

| 场景名称 | 描述 | 障碍物类型 |
|---------|------|-----------|
| 密集动态 | 密集动态障碍物场景 | 静态圆形 + 动态障碍物 |
| 密集动态1 | 密集动态障碍物变体 | 静态圆形 + 动态障碍物 |
| 密集静态 | 密集静态障碍物场景 | 静态圆形 |
| 对向来车 | 对向行驶场景 | 动态障碍物 |
| 横穿 | 横穿障碍物场景 | 动态障碍物 |
| 横穿让行 | 横穿让行场景 | 动态障碍物 |
| 超车 | 超车场景 | 动态障碍物 |
| 跟车 | 跟车场景 | 动态障碍物 |
| 避让后车 | 避让后方车辆场景 | 动态障碍物 |

---

## 📖 使用指南

### 可视化界面说明

程序运行后会打开一个可视化窗口，包含以下元素：

#### 控制按钮（左上角）
- **Pause**：暂停仿真（橙色按钮）
- **Start**：继续仿真（绿色按钮）

#### 场景选择按钮（顶部中间，3x3 网格）
- 点击任意场景按钮可切换到对应场景
- 场景配置文件：`scenarios/scenario_buttons.yaml`
- 支持中文显示场景名称

#### 显示项复选框（右侧）
实时控制各元素的显示/隐藏：
- **Reference Path**：参考路径（黑色虚线）
- **Ego History**：机器人历史轨迹（蓝色实线）
- **Candidate Guidance Path**：候选引导路径
- **Selected Guidance Path**：选中的引导路径
- **MPC Candidates**：MPC 候选轨迹（绿色细线）
- **MPC Best Trajectory**：MPC 最优轨迹（绿色粗线）
- **MPC Selected**：MPC 选中轨迹
- **Static Obstacle**：静态障碍物（红色圆圈）
- **Dynamic Obstacle**：动态障碍物（橙色矩形）
- **Obstacle Prediction**：障碍物预测轨迹
- **Robot Footprint**：机器人轮廓（蓝色矩形）
- **Goal**：目标点（红色星号）
- **Planned Speed**：规划速度（绿色线）
- **History Speed**：历史速度（蓝色线）

**使用方法**：
- 选中（蓝色方块）：显示该元素
- 未选中（白色方块）：隐藏该元素
- 点击复选框可实时切换显示状态

#### 主图（上方）
显示路径规划的 2D 俯视图：
- **蓝色虚线**：参考路径（从起点到终点的直线）
- **蓝色实线**：机器人历史轨迹
- **绿色细线**：MPC 候选轨迹（多条）
- **绿色粗线**：MPC 最优轨迹（选中的最佳轨迹）
- **红色圆圈**：静态障碍物
- **橙色矩形**：动态障碍物
- **蓝色矩形**：机器人轮廓
- **红色星号**：目标点

**坐标范围**：
- 根据起点和终点自动计算固定坐标范围
- 仿真过程中坐标范围不变，避免窗口抖动
- 场景切换时自动重新计算坐标范围

#### 速度图（下方）
显示速度随时间的变化：
- **绿色线**：规划速度（MPC 输出的目标速度）
- **蓝色线**：历史速度（机器人实际速度）

---

## 🎮 场景配置

### 场景文件格式

场景文件使用 JSON 格式，包含以下信息：

```json
{
  "name": "场景名称",
  "startPose": {
    "x": 0.0,
    "y": 0.0,
    "yaw": 0.0
  },
  "goalPose": {
    "x": 20.0,
    "y": 0.0,
    "yaw": 0.0
  },
  "obstacles": {
    "circles": [
      {"x": 10.0, "y": 0.5, "radius": 0.4}
    ],
    "polygons": [],
    "dynamic": [
      {
        "kind": "circle",
        "data": {"r": 0.5},
        "state": {"x": 15.0, "y": 0.0, "vx": -0.5, "vy": 0.0, "yaw": 3.14}
      }
    ]
  }
}
```

### 创建自定义场景

1. **复制模板**：
   ```bash
   cp scenarios/密集动态.json scenarios/my_scenario.json
   ```

2. **编辑场景文件**：
   - 修改起点和终点坐标
   - 添加/删除障碍物
   - 配置动态障碍物的速度

3. **运行场景**：
   ```bash
   ./build_run_sim.sh my_scenario
   ```

### 场景按钮配置

编辑 `scenarios/scenario_buttons.yaml` 可配置场景按钮：

```yaml
buttons:
  - id: 1
    label: "密集动态障碍物场景"
    short_label: "密集动"
    scenario: "scenarios/密集动态.json"
```

详细说明请参考：[scenarios/README.md](scenarios/README.md)

---

## ⚙️ 配置参数

### 主要配置文件

| 配置文件 | 位置 | 说明 |
|---------|------|------|
| 仿真器配置 | `mpc_planner_jackalsimulator/config/` | 机器人参数、控制限制 |
| 求解器配置 | `mpc_planner_solver/config/` | MPC 求解器参数 |
| 引导规划器配置 | `guidance_planner/config/` | RRT* 参数、同伦类配置 |
| 场景配置 | `scenarios/*.json` | 场景起点、终点、障碍物 |
| 场景按钮配置 | `scenarios/scenario_buttons.yaml` | 场景按钮映射 |

### 关键参数说明

#### 1. 控制限制（仿真器配置）
```yaml
limits:
  v_max: 2.0      # 最大线速度 (m/s)
  a_max: 2.0      # 最大线加速度 (m/s²)
  omega_max: 2.0  # 最大角速度 (rad/s)
```

#### 2. MPC 参数（求解器配置）
```yaml
horizon: 20           # 预测时域长度
dt: 0.1              # 时间步长 (s)
max_obstacles: 100   # 最大障碍物数量
```

#### 3. 引导规划器参数
```yaml
max_iterations: 1000  # RRT* 最大迭代次数
goal_tolerance: 0.5   # 目标容差 (m)
```

---

## ❓ 常见问题

### 1. 编译错误：找不到 ACADOS

**问题**：
```
CMake Error: Could not find ACADOS
```

**解决方法**：
- 确保已安装 ACADOS 并设置环境变量
- 检查 `ACADOS_SOURCE_DIR` 是否正确设置
- 运行 `source ~/.bashrc` 重新加载环境变量

### 2. 运行时错误：找不到 Python 模块

**问题**：
```
ModuleNotFoundError: No module named 'matplotlib'
```

**解决方法**：
```bash
pip3 install matplotlib numpy
```

### 3. 中文显示为方框

**问题**：界面中的中文显示为方框

**解决方法**：
```bash
# 安装中文字体
sudo apt-get install -y fonts-wqy-microhei fonts-wqy-zenhei

# 清除 matplotlib 字体缓存
rm -rf ~/.cache/matplotlib
```

### 4. 可视化窗口抖动

**问题**：可视化窗口在仿真过程中抖动

**解决方法**：
- 已在代码中实现固定坐标范围功能
- 如果仍然抖动，请检查是否使用最新版本代码

### 5. 场景加载失败

**问题**：
```
[ERROR] 加载场景文件失败
```

**解决方法**：
- 检查 JSON 文件格式是否正确
- 使用在线 JSON 验证工具检查语法
- 确保文件路径正确

### 6. 障碍物数量过多导致性能问题

**问题**：场景中障碍物过多，求解器超时

**解决方法**：
- 增加 `max_obstacles` 配置参数
- 减少场景中的障碍物数量
- 简化矩形障碍物的圆形近似（修改近似算法参数）

---

## 📚 参考文献

1. **T-MPC++ 算法**：
   - 论文：待补充
   - 相关文档：`docs/tmpc_parallelization_implementation.md`

2. **Contouring 控制**：
   - 论文：待补充
   - 相关文档：`docs/non_guided_planner_explanation.md`

3. **ACADOS**：
   - 官方网站：https://docs.acados.org/
   - GitHub：https://github.com/acados/acados

4. **RRT* 算法**：
   - 论文：Karaman, S., & Frazzoli, E. (2011). Sampling-based algorithms for optimal motion planning.

---

## 📄 许可证

本项目采用 [LICENSE](LICENSE) 许可证。

---

## 🤝 贡献

欢迎贡献代码、报告问题或提出建议！

---

## 📧 联系方式

如有问题或建议，请通过以下方式联系：
- 提交 Issue
- 发送邮件

---

**祝您使用愉快！** 🎉

