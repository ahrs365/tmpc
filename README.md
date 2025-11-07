# MPC Planner - 纯 C++ 实现

采用 T-MPC++ 算法，支持动态障碍物避障和拓扑引导路径规划。无 ROS 依赖，可独立运行。在原项目的基础上，去除了ros依赖，并增加了场景加载模块，可以加载不同场景进行测试。

本规划器可以作为[navsim-local](https://github.com/ahrs365/navsim-local)的一个规划器插件进行使用，也可单独使用

场景可通过在线场景编辑器进行生成：[地址](https://www.gl-robotics.com/navsim-online/index.html)

![alt text](docs/image-6.png)

9个场景仿真效果：

![<video controls src="docs/tmpc效果展示.mp4" title="Title"></video>](docs/效果展示.gif)

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

## 编译运行

```bash
# 方法 1：使用便捷脚本（推荐）
./build_run_sim.sh

# 使用场景名称
./build_run_sim.sh 密集动态
```

---

## 🎯 项目简介


### 核心算法：T-MPC++

T-MPC++（Topology-guided Model Predictive Control）是一种结合拓扑引导和并行优化的先进 MPC 算法：

1. **拓扑引导**：使用引导路径规划器（Guidance Planner）生成多条不同拓扑类别的候选路径
2. **并行优化**：为每条引导路径并行运行独立的 MPC 求解器
3. **最优选择**：从所有成功的候选轨迹中选择代价最小的作为最终控制输出

---



### T-MPC++ 算法流程

```
1. 引导路径规划
   └─> 同伦类分类生成多条候选引导路径

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

支持自行车模型和二阶单轮车模型（Second-Order Unicycle Model），默认采用单轮模型：

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
│   ├── src/                     # 同伦类分类实现
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

## 💻 系统要求

### 操作系统
- Ubuntu

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

## 快速开始

```bash
# 方法 1：使用便捷脚本（推荐）
./build_run_sim.sh

# 使用场景名称
./build_run_sim.sh 密集动态
```

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

## 参考项目和文献：

```
[1] Journal Paper: O. de Groot, L. Ferranti, D. M. Gavrila, and J. Alonso-Mora, Topology-Driven Parallel Trajectory Optimization in Dynamic Environments. IEEE Transactions on Robotics (T-RO) 2024. Available: https://doi.org/10.1109/TRO.2024.3475047

```

关注公众号，获取更多内容：

![alt text](docs/微信公众号.jpg)