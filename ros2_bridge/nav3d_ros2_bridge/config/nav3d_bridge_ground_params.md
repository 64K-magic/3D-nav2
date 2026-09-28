# `planning_traversability:=ground` 参数说明

对应配置文件：[`nav3d_bridge.yaml`](./nav3d_bridge.yaml)。

本文只说明在 **地面支撑可通行（ground）** 下有意义、会被用到、或会被代码改写/忽略的参数。  
启动时通常用 launch 覆盖：

```bash
ros2 launch nav3d_ros2_bridge nav3d_bridge.launch.py \
  planning_traversability:=ground \
  planning_mode:=3d \
  ...
```

yaml 里默认的 `planning.traversability: uav` 会被上述 launch 参数覆盖。

---

## 1. 参数分类总览

| 类别 | 参数 | ground 下行为 |
|------|------|----------------|
| **Ground 专用（核心）** | `planning.ground_*` | 支撑、净空、起终点吸附 |
| **规划通用（建议 ground 一起配）** | `planning.mode`、`search_algorithm`、动态可行性等 | 仍生效；搜索走 ground A* |
| **地图 / 分辨率** | `map.*` | 决定格子大小；单步爬升 ≈ 1×resolution |
| **控制器（ground 会改写部分值）** | `controller.*` | 限速/短前瞻；非 uav 运动模型会被强制成 uav |
| **安全** | `safety.*` | **轨迹前方碰撞检测在 ground 下被跳过** |
| **Ground 下忽略 / 基本无效** | `planning.inflation_radius`、`planning.uav_endpoint_snap_radius_cells` | 膨胀不用；UAV snap 不用 |

---

## 2. Ground 专用核心参数

### `planning.traversability`

- **类型**：`string`，`uav` | `ground`
- **yaml 默认**：`uav`
- **ground 含义**：启用「脚下要有 occupied 支撑 + 身体净空」的可通行模型；悬空格不可走，禁止空中捷径。
- **说明**：本文件讨论的模式；实际运行以 launch/`ros__parameters` 最终值为准。

### `planning.ground_strict_direct_support`

- **类型**：`bool`
- **默认**：`false`
- **含义**：
  - `false`：在「向下 `ground_support_depth_cells` 层、水平 `ground_support_xy_radius_cells` 脚印」内任意格找到 occupied 即算有支撑。
  - `true`：只认**正下方一格** occupied（更严，斜踏/稀疏点云更容易判无支撑）。
- **调参建议**：楼梯、点云稀疏时保持 `false`；只要「必须踩在正下方实体上」再开 `true`。

### `planning.ground_snap_radius_cells`

- **类型**：`int`（格）
- **默认**：`6`
- **含义**：点击 start/goal 时，若当前格不可走，在切比雪夫半径 ≤ 该值的壳层内搜索最近可走格并吸附。
- **物理尺度**：约 `snap_radius_cells × map.resolution`（0.2 m 时约 1.2 m）。
- **注意**：半径过大时 snap 更慢；过小则点击稍偏就吸附失败。

### `planning.ground_support_xy_radius_cells`

- **类型**：`int`（格）
- **默认**：`1`
- **含义**：判断「脚下有没有地」时，在水平方向允许的脚印半径（含对角线邻格）。`1` → 查 3×3 柱脚。
- **作用**：踏面体素略偏、不完全在正下方时仍可判为可站。

### `planning.ground_support_depth_cells`

- **类型**：`int`（格）
- **默认**：`1`
- **含义**：从站立格向下最多查几层 occupied。`1` → 只看下一格（约一个 resolution）。
- **不是**「允许爬多高台阶」。单步爬升仍由邻居 `|Δz|≤1` 限制（≈ 1×resolution）。
- **作用**：禁止悬空；深度过大可能把离地较远的格也判成有支撑（更像“漂”）。

### `planning.ground_robot_radius`

- **类型**：`double`（米）
- **默认**：`0.25`
- **含义**：站立格处身体净空球半径。球内（主要同层及上方邻格）若有 occupied → 不可走。
- **与膨胀关系**：ground 模式**不使用** `planning.inflation_radius`；想加大障碍余量，应调本参数，而不是 inflation。
- **调参**：机器人半宽/半对角线量级；过大 → 窄通道/楼梯过不去；过小 → 贴墙擦障。

---

## 3. 规划相关（ground 仍生效）

### `planning.mode`

- **类型**：`string`，`2d` | `3d`
- **默认**：`3d`
- **ground + 3d**：邻居可含竖直 `|Δz|≤1`，可爬楼。
- **ground + 2d**：不允许变 z，只能同层地面走（楼梯场景一般用 `3d`）。

### `planning.search_algorithm`

- **类型**：`string`，`astar` | `jps`
- **默认**：`astar`
- **说明**：配置会读入；ground 路径搜索当前走的是桥接内的 **ground-supported A\***（带支撑约束）。以实际 planner 接线为准；爬楼场景建议保持 `astar`。

### `planning.use_octomap_map`

- **类型**：`bool`
- **默认**：`false`
- **含义**：`false` 用稠密 `VoxelGridMap` 规划；`true` 用 OctoMap 叶子作全局规划底图。
- **ground 建议**：默认 `false` 更稳、行为更直观。

### `planning.inflation_radius`

- **类型**：`double`（米）
- **默认**：`0.5`
- **ground 下**：**不生效**（代码在 ground 时跳过 `InflationLayer`，避免地板向上膨胀把踏面抬到断开层）。
- **UAV 下**：作为自由空间机体膨胀半径。
- **ground 替代**：使用 `planning.ground_robot_radius`。

### `planning.uav_endpoint_snap_radius_cells`

- **类型**：`int`
- **默认**：`12`
- **ground 下**：**不使用**（起终点走 `resolveGroundEndpoint` / ground snap）。
- 仅 UAV 模式吸附自由空间时用。

### 动态可行性一组

| 参数 | 默认 | 含义（简要） |
|------|------|----------------|
| `planning.enable_dynamic_feasibility_check` | `false` | 是否对轨迹做速度/加速度可行性检查 |
| `planning.dynamic_feasibility_mode` | `sampled` | 检查方式：`sampled` / `analytic_bounds` / `analytic_exact` |
| `planning.max_velocity` | 极大 | 动力学速度上限 |
| `planning.max_acceleration` | 极大 | 动力学加速度上限 |
| `planning.feasibility_sample_dt` | `0.05` | 采样检查时间步（秒） |
| `planning.max_dynamic_time_scale` | `10.0` | 不可行时最多把时间轴拉长的倍数 |
| `planning.enable_initial_time_allocation` | `false` | 是否做初始时间分配 |
| `planning.min_time_allocation_interval` | `0.05` | 时间分配最小间隔（秒） |

ground 爬楼 demo 通常保持动力学检查关闭（当前 yaml 即如此）；打开后需按真实机器人限速限加速度设置，否则易 plan_failed 或过度减速。

---

## 4. 地图参数（决定格子与单步台阶高度）

### `map.resolution`

- **类型**：`double`（米）
- **默认**：`0.2`
- **ground 关键**：
  - 体素边长；
  - A* 单次扩展 `|Δz|≤1` → **单步最大爬升 ≈ resolution**（0.2 m → 约 20 cm）；
  - `support_*_cells`、`snap_radius_cells`、`ground_robot_radius` 换算成格数都依赖它。

### `map.pcd_path` / `map.pcd_loader`

- 点云路径与加载器（`builtin` / `pcl`）。launch 里的 `pcd_path:=...` 会覆盖空默认。

### `map.min_points_per_voxel` / `map.min_cluster_voxels`

- 预处理：格内最少点数、最小连通体素数。过大易丢薄踏面；过小噪声更多。

### `map.insert_free_space_rays` + `map.sensor_origin.*`

- 是否从传感器原点向命中点插 free 射线。ground 默认 `false`：只标 occupied，未知区在本实现里常被当可走（再加支撑约束）。

### `map.occupancy_grid_min_z` / `max_z` / `max_cells`

- 仅影响 **2D 占据栅格可视化** `/nav3d/occupied_grid` 的高度切片与上限，不直接改 3D ground 搜索规则。

---

## 5. 局部地图 `local_grid.*`

局部观测叠加到规划图（若启用且有 `/nav3d/local_pointcloud`）。

| 参数 | 含义 |
|------|------|
| `local_grid.enabled` | 是否启用局部栅格 |
| `local_grid.resolution` | 局部分辨率 |
| `local_grid.ray_clearing.enabled` | 是否射线清障 |
| `local_grid.retention_sec` | 帧保留时间（0 表示按帧数策略） |
| `local_grid.max_retained_frames` | 最多保留帧数 |
| `local_grid.max_observation_age_sec` | 观测最大年龄 |
| `local_grid.max_future_stamp_sec` | 允许时间戳超前量 |

纯 PCD + pose_sim、无实时点云时，对全局 ground 规划影响很小。

---

## 6. 安全 `safety.*`（ground 下大部分前方碰撞逻辑被跳过）

| 参数 | 含义 | ground 注意 |
|------|------|-------------|
| `safety.enabled` | 是否构造安全监控 | ground 在控制循环里**直接 return**，不跑轨迹前方碰撞/急停那套 |
| `safety.emergency_stop_horizon` | 急停时间窗（秒） | UAV 有效；ground 基本用不上 |
| `safety.lookahead_horizon` | 前瞻碰撞时间窗 | 同上 |
| `safety.replan_cooldown_sec` | 重规划冷却 | 同上 |
| `safety.local_replan_*` | 局部 A* 重规划 | 主要为 UAV 自由空间场景 |

楼梯路径 B 样条弦线易“擦”立板，故 ground 故意不因 ahead-collision 急停，避免停走抖动。

---

## 7. 控制器 `controller.*`（ground 会改写）

| 参数 | yaml 默认 | ground 行为 |
|------|-----------|-------------|
| `controller.enabled` | `true` | 是否发 `cmd_vel` |
| `controller.control_dt` | `0.1` | 控制周期（秒） |
| `controller.lookahead_time` | `0.4` | 被 **上限压到 ≤ 0.25** |
| `controller.max_speed` | `1.0` | 被 **上限压到 ≤ 0.55** |
| `controller.goal_tolerance` | `0.15` | 到点判定（米） |
| `controller.command_frame` | `map` | `map` / `body` |
| `controller.motion_model` | `uav` | 若设成 omni/diff/ackermann，ground 会 **强制改成 uav**（需要 `linear.z` 爬楼） |
| `controller.max_linear_acceleration` 等 | 见 yaml | 加速度、最小转弯半径、偏航增益/限速等 |
| `controller.block_occupied_voxels` | 未写则自动 | ground **默认 false**（楼梯弦线易穿立板，挡住会卡死） |
| `controller.command_path_sample_step` | `0.05` | 路径采样步长（米） |

---

## 8. 其它与起终点 / 轨迹显示

| 参数 | 含义 |
|------|------|
| `frame_id` | TF/消息坐标系，需与 RViz Fixed Frame 一致 |
| `start.x/y/z` | 节点初始起点（building2_9 默认楼梯平台）；点击 `/nav3d/start` 会覆盖 |
| `trajectory.publish_max_poses` | 发布轨迹最多点数 |

`nav3d_cmd_vel_pose_sim` 段为仿真位姿节点参数（`start_*`、`update_rate`、`max_step`、`cmd_timeout` 等），与 traversability 模式独立，但 ground 爬楼 demo 通常一并启动。

---

## 9. Ground 调参速查

| 目标 | 优先调 |
|------|--------|
| 一步能爬的台阶高度 | `map.resolution`（单步 ≈ 1 格） |
| 更贴墙/更危险 | 减小 `ground_robot_radius` |
| 更保守离障 | 增大 `ground_robot_radius`（不要指望 inflation） |
| 点击吸附更远/更近 | `ground_snap_radius_cells` |
| 踏面略偏仍算有地 | 增大 `ground_support_xy_radius_cells` 或保持 strict=false |
| 更严“正下方才算地” | `ground_strict_direct_support: true` |
| 爬楼速度/平滑 | `controller.max_speed` / `lookahead_time`（仍受 ground 上限夹紧） |

---

## 10. 与 UAV 对照（同文件参数）

| 能力 | ground | uav |
|------|--------|-----|
| 脚下支撑 | 要 | 不要 |
| `inflation_radius` | 忽略 | 生效 |
| 起终点吸附 | `ground_snap_*` + support | `uav_endpoint_snap_radius_cells` |
| 安全 ahead 碰撞 | 跳过 | 启用 |
| 控制器 | 限速 + 强制可出 vz | 按配置运动模型 |
