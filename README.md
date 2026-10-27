# AIC-2025 省赛 · 自主导航移动机器人（ROS1 全栈实现）

面向 2025 年 AIC 竞赛省赛任务的差速移动机器人软件系统。基于 **ROS1 (Melodic/Noetic) + catkin** 构建，自研**激光雷达扫描匹配定位节点**与**局部规划器插件**，替换掉默认的 AMCL 与 DWA，实现建图、定位、导航与目标识别的完整闭环。

> 本仓库仅包含自研代码与配置。第三方 YOLO 检测包（`yolo_ros`，含 ultralytics 子模块与模型权重）体积超过 100MB，未纳入版本管理，依赖说明见下文。

## 系统架构

```
              ┌──────────────── move_base ────────────────┐
              │  global_planner: CarrotPlanner / GlobalPlanner │
   /map ──────┤                                           │
              │  local_planner: SimpleLocalPlannerROS（自研）  │──► /cmd_vel
              └───────────────────────────────────────────┘
                    ▲                          ▲
                    │ /map→/odom (TF)          │ /scan
         ┌──────────┴──────────┐      ┌────────┴─────────┐
         │  jie_ware/lidar_loc │      │ jie_ware/lidar_filter_node │
         │  扫描匹配定位（自研） │      │  点云滤波（自研）  │
         └──────────┬──────────┘      └────────┬─────────┘
                    │ /scan                    │ /scan_filtered
              ┌─────┴──────────────────────────┴─────┐
              │        YDLIDAR X2 激光雷达            │
              └──────────────────────────────────────┘

  仿真侧：robot_simulation (Gazebo world + 机器人模型)
  描述侧：robot_description (URDF + STL meshes)
  建图侧：gmapping / hector / karto / cartographer 可选
```

## 功能包说明

| 功能包 | 类型 | 说明 |
|---|---|---|
| `jie_ware` | 自研 | 三个 C++ 节点的集合，本项目的核心算法所在 |
| `simple_local_planner` | 自研 | 继承 `nav_core::BaseLocalPlanner` 的局部规划器插件 |
| `robot_navigation` | 配置为主 | move_base / SLAM / 地图 的 launch 与参数组织 |
| `robot_simulation` | 配置为主 | Gazebo 世界、机器人模型、材质贴图 |
| `robot_description` | 配置为主 | URDF 模型与 STL 网格（差速底盘、雷达、IMU） |

### 自研节点 1：`lidar_loc` — 激光扫描匹配定位（504 行）

替代 AMCL，直接以「当前帧激光点 vs 先验栅格地图」做匹配求位姿。

- 订阅 `/map`（`OccupancyGrid`）并转化为 `cv::Mat`，按 ROI 裁切出有效区域加速匹配
- 订阅 `/scan`（`LaserScan`），把极坐标点转换为地图栅格坐标系下的点集
- 在初始位姿附近做**多分辨率搜索 + 变换评分**，取 `best_transform` 作为最优位姿
- 将位姿发布为 `map → odom` 的 TF 变换，供 `move_base` 与 RViz 使用
- 位姿跳变超阈值时调用 `clear_costmaps` 服务清理代价地图，避免导航卡死
- 支持通过 `/initialpose` 话题手动重定位

关键点：这是把「点云配准」问题直接落到栅格地图上做，省掉 AMCL 的粒子滤波开销，在嵌入式平台上实时性更好。

### 自研节点 2：`lidar_filter_node` — 激光点云滤波（142 行）

- 滤除机体自身遮挡产生的固定角度噪声点
- 剔除超出有效量程与强度阈值的点
- 输出 `/scan_filtered` 供定位与代价地图使用，降低误匹配概率

### 自研节点 3：`costmap_cleaner` — 代价地图清理（40 行）

在机器人被卡住或定位跳变时主动触发 `move_base/clear_costmaps`，配合 `lidar_loc` 的跳变检测做导航自恢复。

### 自研插件：`SimpleLocalPlannerROS`（256 行）

一个轻量的纯追踪（Pure Pursuit）式局部规划器，通过 `pluginlib` 注册进 `move_base`。

四状态机设计：

| 状态 | 行为 |
|---|---|
| `INITIAL_ROTATION` | 原地旋转对准路径起点方向 |
| `FOLLOWING_PATH` | 按 `carrot_distance` 取前视点，计算线速度/角速度跟踪 |
| `FINAL_ROTATION` | 到达位置后原地旋转对准目标朝向 |
| `GOAL_REACHED` | 置零速度，上报到达 |

核心参数（`param/simple_local_planner.yaml`）：

```yaml
yaw_tolerance: 0.15        # rad，角度容差
xy_goal_tolerance: 0.15    # m，位置容差
carrot_distance: 0.2       # m，前视点距离
max_linear_speed: 0.5      # m/s
max_angular_speed: 0.8     # rad/s
```

可插拔设计：`navigation.launch` 提供 `planner:=dwa|teb|simple` 三个开关，方便与官方规划器做横向对比测试。

## 目录结构

```
.
├── catkin_ws/
│   └── src/
│       ├── jie_ware/                 # 自研：lidar_loc / lidar_filter_node / costmap_cleaner
│       │   ├── src/
│       │   └── launch/
│       ├── simple_local_planner/     # 自研：nav_core 局部规划器插件
│       │   ├── include/
│       │   ├── src/
│       │   └── param/
│       ├── robot_navigation/         # move_base / SLAM / 地图 / 代价地图参数
│       │   ├── launch/               # navigation / move_base / slam_laser / map_saver
│       │   ├── param/                # params_nav_common / params_costmap_common
│       │   └── map/                  # 建图结果（.pgm + .yaml）
│       ├── robot_simulation/         # Gazebo 世界与机器人模型
│       └── robot_description/        # URDF + STL 网格
└── docs/
    └── AIC-2025-64302102-技术文档.pdf   # 参赛技术文档（系统设计、实验数据、结论）
```

## 硬件平台

| 项目 | 型号 |
|---|---|
| 主控 | 上位机运行 ROS（Ubuntu 18.04/20.04） |
| 底盘 | ESP32 差速底盘（经 `chassis_bringup` 驱动包接入） |
| 激光雷达 | YDLIDAR X2（360°，经 `ydlidar_ros_driver` 接入） |
| 视觉 | USB 摄像头 + YOLO 目标检测 |

## 编译与运行

```bash
# 1. 依赖
sudo apt install ros-$ROS_DISTRO-navigation ros-$ROS_DISTRO-gmapping \
     ros-$ROS_DISTRO-teb-local-planner ros-$ROS_DISTRO-move-base \
     ros-$ROS_DISTRO-cartographer ros-$ROS_DISTRO-cv-bridge \
     ros-$ROS_DISTRO-tf2-geometry-msgs ros-$ROS_DISTRO-costmap-2d
# 另需 OpenCV（3.x/4.x）

# 2. 编译
cd catkin_ws
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash

# 3. 建图（可选 gmapping / hector / karto / cartographer）
roslaunch robot_navigation slam_laser.launch slam_methods:=gmapping

# 4. 导航（默认使用自研局部规划器）
roslaunch robot_navigation navigation.launch planner:=simple simulation:=false

# 5. 对比测试官方规划器
roslaunch robot_navigation navigation.launch planner:=teb
```

`simulation:=true` 时走 Gazebo 仿真，`false` 时启动 ESP32 底盘与 YDLIDAR X2 实机驱动。

## 第三方依赖（未纳入仓库）

| 依赖 | 用途 | 获取方式 |
|---|---|---|
| `yolo_ros` + `ultralytics` | 目标检测（竞赛识别任务） | 上游开源包 + 自训练权重 `best.pt` |
| `chassis_bringup` | ESP32 底盘驱动 | 底盘厂商提供 |
| `ydlidar_ros_driver` | YDLIDAR X2 雷达驱动 | 雷达厂商提供 |

> 自训练权重约 5.5MB，如需用于复现请联系作者。

## 说明

- 作者：张涛（物联2311），参赛队伍：少年先疯队，参赛编号 64302102
- 仓库已剔除 `build/`、`devel/`、`.git`、模型权重等非源码内容
- 详细系统设计、参数标定过程与实验数据见 `docs/` 下的技术文档
