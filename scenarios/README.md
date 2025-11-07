# MPC Planner 场景配置文件

本文件夹包含用于 MPC Planner 仿真的场景配置文件（JSON 格式）。

## 📋 使用方法

### **方法 1：使用便捷脚本（推荐）**

```bash
# 运行指定场景（只需输入场景名称，如 s1、s2 等）
./build_run_sim.sh s1

# 运行默认场景（不使用 JSON 文件）
./build_run_sim.sh
```

### **方法 2：直接运行可执行文件**

```bash
cd build
# 运行指定场景
./mpc_planner_main ../mpc_planner_jackalsimulator/config ../scenarios/s1.json

# 运行默认场景
./mpc_planner_main ../mpc_planner_jackalsimulator/config
```

---

## 📁 场景文件列表

| 文件 | 描述 | 障碍物类型 | 障碍物数量 |
|------|------|-----------|-----------|
| `s1.json` | 静态圆形障碍物场景 | 静态圆形 | 19 个 |
| `s2.json` | 多边形障碍物场景 | 静态圆形 + 多边形 | 1 圆 + 8 多边形 → 47 个圆 |
| `s3.json` | 动态矩形障碍物场景 | 动态矩形 | 3 个矩形 → 18 个圆 |
| `s4.json` | 动态圆形障碍物场景 | 动态圆形 | 3 个 |
| `s5.json` | 混合动态障碍物场景 | 动态圆形 + 矩形 | 2 圆 + 1 矩形 |
| `s6.json` | 单个动态障碍物场景 | 动态圆形 | 1 个 |
| `s7.json` | 复杂动态场景 | 动态矩形 | 19 个矩形 |

---

## 📝 JSON 文件格式说明

### **完整格式示例**

```json
{
  "name": "scenario_name",
  "timestamp": 1762479959964,
  
  "startPose": {
    "x": 0.0,
    "y": 0.0,
    "yaw": 0.0
  },
  
  "goalPose": {
    "x": 20.0,
    "y": 0.0,
    "yaw": 0.0,
    "tol": {
      "pos": 0.2,
      "yaw": 0.1
    }
  },
  
  "obstacles": {
    "circles": [
      {
        "x": 10.0,
        "y": 0.5,
        "radius": 0.4
      }
    ],
    
    "polygons": [
      {
        "points": [
          {"x": 5.0, "y": 1.0},
          {"x": 6.0, "y": 1.0},
          {"x": 6.0, "y": 2.0},
          {"x": 5.0, "y": 2.0}
        ],
        "closed": true
      }
    ],
    
    "dynamic": [
      {
        "kind": "circle",
        "data": {
          "type": "circle",
          "r": 0.5
        },
        "state": {
          "x": 15.0,
          "y": 0.0,
          "vx": -0.5,
          "vy": 0.0,
          "yaw": 3.14159
        }
      },
      {
        "kind": "rect",
        "data": {
          "type": "rect",
          "w": 1.0,
          "h": 0.5,
          "yaw": 0.0
        },
        "state": {
          "x": 18.0,
          "y": 1.0,
          "vx": -0.3,
          "vy": 0.0,
          "yaw": 0.0
        }
      }
    ]
  },
  
  "chassisType": "differential",
  "chassisConfig": {
    "model": "differential",
    "wheelbase": 0,
    "track_width": 0.4,
    "limits": {
      "v_max": 2,
      "a_max": 2,
      "omega_max": 2,
      "steer_max": 0
    }
  }
}
```

---

## 🔧 字段说明

### **1. 起点和终点**

| 字段 | 类型 | 必需 | 说明 |
|------|------|------|------|
| `startPose.x` | number | ✓ | 起点 X 坐标 (m) |
| `startPose.y` | number | ✓ | 起点 Y 坐标 (m) |
| `startPose.yaw` | number | ✓ | 起点朝向 (rad) |
| `goalPose.x` | number | ✓ | 终点 X 坐标 (m) |
| `goalPose.y` | number | ✓ | 终点 Y 坐标 (m) |
| `goalPose.yaw` | number | ✗ | 终点朝向 (rad) |
| `goalPose.tol.pos` | number | ✗ | 位置容差 (m) |
| `goalPose.tol.yaw` | number | ✗ | 朝向容差 (rad) |

### **2. 静态圆形障碍物**

```json
"circles": [
  {
    "x": 10.0,      // 圆心 X 坐标 (m)
    "y": 0.5,       // 圆心 Y 坐标 (m)
    "radius": 0.4   // 半径 (m)
  }
]
```

### **3. 静态多边形障碍物**

```json
"polygons": [
  {
    "points": [
      {"x": 5.0, "y": 1.0},
      {"x": 6.0, "y": 1.0},
      {"x": 6.0, "y": 2.0},
      {"x": 5.0, "y": 2.0}
    ],
    "closed": true
  }
]
```

**注意**：多边形会被自动近似为多个圆形障碍物。

### **4. 动态障碍物**

#### **动态圆形障碍物**

```json
{
  "kind": "circle",
  "data": {
    "type": "circle",
    "r": 0.5          // 半径 (m)
  },
  "state": {
    "x": 15.0,        // 初始 X 坐标 (m)
    "y": 0.0,         // 初始 Y 坐标 (m)
    "vx": -0.5,       // X 方向速度 (m/s)
    "vy": 0.0,        // Y 方向速度 (m/s)
    "yaw": 3.14159    // 朝向 (rad)
  }
}
```

#### **动态矩形障碍物**

```json
{
  "kind": "rect",
  "data": {
    "type": "rect",
    "w": 1.0,         // 宽度 (m)
    "h": 0.5,         // 高度 (m)
    "yaw": 0.0        // 朝向 (rad)
  },
  "state": {
    "x": 18.0,        // 初始 X 坐标 (m)
    "y": 1.0,         // 初始 Y 坐标 (m)
    "vx": -0.3,       // X 方向速度 (m/s)
    "vy": 0.0,        // Y 方向速度 (m/s)
    "yaw": 0.0        // 朝向 (rad)
  }
}
```

**注意**：矩形障碍物会被自动近似为多个圆形障碍物。

---

## 🎯 矩形障碍物近似算法

### **自适应覆盖算法**

程序使用自适应网格覆盖算法将矩形近似为多个圆形：

1. **圆半径选择**：`radius = min(width, height) × 0.35`
2. **网格数量计算**：
   - X 方向：`nx = ceil(width / (2 × radius))`
   - Y 方向：`ny = ceil(height / (2 × radius))`
3. **圆形分布**：在矩形内均匀分布 `nx × ny` 个圆
4. **旋转变换**：根据矩形朝向 `yaw` 旋转所有圆的位置

### **示例**

```
矩形: 1.3m × 0.65m, yaw=0
→ 半径: 0.227m
→ 网格: 3 × 2 = 6 个圆
```

---

## 🚀 创建自定义场景

### **步骤 1：复制模板**

```bash
cp scenarios/s1.json scenarios/my_scenario.json
```

### **步骤 2：编辑场景**

使用文本编辑器修改 `my_scenario.json`：

```json
{
  "name": "my_scenario",
  "startPose": {"x": 0, "y": 0, "yaw": 0},
  "goalPose": {"x": 30, "y": 0, "yaw": 0},
  "obstacles": {
    "circles": [
      {"x": 10, "y": 1, "radius": 0.5},
      {"x": 20, "y": -1, "radius": 0.4}
    ],
    "polygons": [],
    "dynamic": [
      {
        "kind": "circle",
        "data": {"r": 0.6},
        "state": {"x": 15, "y": 0, "vx": -0.5, "vy": 0, "yaw": 3.14}
      }
    ]
  }
}
```

### **步骤 3：运行场景**

```bash
# 使用便捷脚本（推荐）
./build_run_sim.sh my_scenario

# 或直接运行可执行文件
cd build
./mpc_planner_main ../mpc_planner_jackalsimulator/config ../scenarios/my_scenario.json
```

---

## 📊 场景统计

运行场景时，程序会输出加载信息：

```
[INFO] Loading scenario from: ../scenarios/s3.json
[INFO] ✓ 从场景文件加载起点: (-0.251369, 0.0466488, -0.0285994)
[INFO] ✓ 从场景文件加载终点: (37.3911, -0.264977)
[INFO]   矩形近似: 1.29733x0.648667 -> 6 个圆 (半径=0.227033)
[INFO]   矩形近似: 0.903816x0.451908 -> 6 个圆 (半径=0.158168)
[INFO]   矩形近似: 1.50527x0.752634 -> 6 个圆 (半径=0.263422)
[INFO] ✓ 加载了 3 个动态障碍物
[INFO] 场景加载完成，共 18 个障碍物圆形
```

---

## ⚠️ 注意事项

1. **坐标系**：使用右手坐标系，X 轴向右，Y 轴向上
2. **角度单位**：所有角度使用弧度（rad）
3. **速度单位**：m/s
4. **障碍物近似**：多边形和矩形会被自动转换为多个圆形
5. **动态障碍物运动**：当前实现为恒速直线运动（未来可扩展为边界反弹等）
6. **JSON 格式**：确保 JSON 文件格式正确，可使用在线工具验证

---

## 🔍 故障排查

### **场景文件未找到**

```
[ERROR] Scenario file not found: ../scenarios/xxx.json
```

**解决方法**：检查文件路径是否正确

### **JSON 解析失败**

```
[ERROR] 加载场景文件失败: ...
[WARN] 使用默认障碍物配置
```

**解决方法**：
1. 使用 JSON 验证工具检查文件格式
2. 确保所有必需字段都存在
3. 检查数值类型是否正确（不要用字符串表示数字）

### **障碍物数量过多导致性能问题**

如果场景中障碍物过多（>100 个圆），可能导致：
- QP 求解器迭代次数增加
- 控制循环超时

**解决方法**：
1. 减少多边形/矩形的近似圆数量（修改 `approximateRectangle` 中的 `base_radius` 系数）
2. 简化场景，移除不必要的障碍物
3. 增加 QP 求解器最大迭代次数（需重新生成求解器）

---

## 📚 相关文档

- [配置文件指南](../docs/configuration_guide.md)
- [可视化增强文档](../docs/visualization_enhancements.md)
- [固定窗口大小文档](../docs/fixed_window_size.md)

