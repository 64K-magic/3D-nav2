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
| **Ground 专用（核心）** | `planning.ground_*`、`imu.*` | 支撑、球/3D footprint、转角**硬核+软环**、贴墙代价、可选平台 via、俯仰估计 |
| **规划通用（建议 ground 一起配）** | `planning.mode`、`search_algorithm`、动态可行性等 | 仍生效；搜索走 ground A*；**跳过三次 B 样条**，沿折线跟踪 |
| **地图 / 分辨率** | `map.*` | 决定格子大小；单步爬升 ≈ 1×resolution |
| **控制器（ground 会改写部分值）** | `controller.*` | 限速/短前瞻；非 uav 运动模型会被强制成 uav |
| **安全** | `safety.*` | **轨迹前方碰撞检测在 ground 下被跳过** |
| **Ground 下忽略 / 基本无效** | `planning.inflation_radius`、`planning.uav_endpoint_snap_radius_cells` | 膨胀不用；UAV snap 不用 |

---

## 2. Ground 规划流水线（先读这段）

```
起终点 snap（支撑+球净空；RViz z=0 自动上抬）
  → ground A*（禁止边走边爬；转角硬核+软环；贴墙代价；爬升惩罚；可选平台 via）
  → 路径跟随轨迹（degree-1，不做三次 B 样条平滑）
  → footprint 检查（路径/跟踪用地形俯仰；跟踪硬停）
```

要点：

1. **A\*** 单元格只用 `ground_robot_radius` 球；完整狗身盒不进 A*（否则楼梯易 `no_path`）。
2. **禁止同一步既水平移动又爬升**；楼梯靠「先竖直再水平」连通，并对 `|Δz|` 加 `ground_climb_penalty`。
3. **转角软环 + 贴墙代价（无需坐标）**：自动检测楼梯区内角 L，只加软代价（避免硬封平台导致 `no_path`）；手工 `keepout_xyr` 才可硬禁；贴墙代价推离栏杆。
4. **平台航点（可选）**：默认关闭；仅在需要时手工配置 `ground_landing_via_points`。
5. **不做三次 B 样条**：避免平滑弦线再次斜穿楼梯井。
6. **俯仰**：优先用地图 footprint 前后地形高差（正 pitch = 抬头，肚皮贴楼梯坡）；路径坡度仅弱混合。

---

## 3. Ground 专用核心参数

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
- **注意**：半径过大时 snap 更慢；过小则点击稍偏就吸附失败。起终点 snap **不**套用转角禁入（避免站在内角附近无法设点）。
- **RViz**：`2D Pose Estimate` / `2D Goal Pose` 固定发 `z=0`，往往落在楼板 occupied 内；ground 会自动向上吸附到第一层可站立自由格。若日志出现 `ground_endpoint_snap_failed`，增大本参数。

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
- **yaml 当前值**：`0.25`
- **含义**：站立格处身体净空球半径。球内（主要同层及上方邻格）若有 occupied → 不可走。
- **与膨胀关系**：ground 模式**不使用** `planning.inflation_radius`。
- **与 footprint**：轨迹/跟踪用 3D footprint（半宽约 0.4 m）；A* **不能**把球半径调到半宽——`building2_9` 上 `≥0.30` 会掐断走廊（起点连通域约 2500 格，`no_path`）。贴栏杆/转角靠 `corner_keepout_penalty` + `wall_clearance`。
- **调参**：走廊 `no_path` / `closed≈两千` → **减小**到 `0.20~0.25`；仅在开阔地图才可略增。

### `planning.ground_climb_penalty`

- **类型**：`double`（无量纲，加在格子代价上）
- **yaml 当前值**：`0.85`
- **含义**：A* 每步代价中与高度相关的项：`climb_penalty × |Δz|`（`|Δz|` 为格子索引差，通常 0 或 1）。增大则更偏好「先走平台平路再爬」，减小则更短、更陡。
- **与转角代价关系**：爬升惩罚抑制「堆叠竖直捷径」；转角 `corner_keepout_penalty` 抑制「贴内角水平+爬升拓扑」。两者常一起调。
- **调参**：仍斜穿楼梯井时略增大（如 `1.0~1.5`）；路径无故绕远或爬楼段被拆得过碎时略减小。

### `planning.ground_leave_floor_penalty`

- **类型**：`double`（无量纲，加在格子代价上）
- **yaml 当前值**：`25.0`
- **含义**：当 `|goal.z − start.z| ≤ 0.6`（看起来同层）时，对落在起终点 z 带外的格子加软代价：`penalty × |z−band| / resolution`。有平路时偏好留在同层；**不做硬截断**。
- **为何不用硬 z 带**：`building2_9` 楼梯平台 `(≈8.5,−4,0.3)` 与西翼 `(≈−6,−4,0.3)` 在 L1 **不连通**，唯一通路经上层（路径 z 可到 ≈13.7）。硬带会 `no_path`（`closed≈2950`），而起终点本身都可走。
- **日志**：成功时打印 `z=[...]` 与 `leave_floor=N`；若 `N>0` 会再打 `same_floor_endpoints_require_climb`。
- **调参**：同层可选平路仍去爬楼 → 增大；同层岛屿必须上楼却搜不到 → 减小或设 `0`。

---

### 转角偏好（平台外绕，**硬核 + 软环**，无需坐标）

楼梯栏杆内角若按最短路贴角走，长机身 footprint 会卡在转角。当前默认策略：

1. **自动严格凹角 L = 距离加权软代价**：仅当正交 occupied 恰好 2 个、实心对角、对向开放时才记种子（旧的「任意两边有墙」会把楼梯栏杆旁几乎每格都标成种子，代价场变平，禁行等于无效）。代价峰值 `penalty × (1-d/r)²`。**不做硬封锁**（硬封会出现 `no_path`）。
2. **贴墙代价**（`wall_clearance`）：中心线靠近 occupied 时加代价。
3. **硬核**（`hard_core_scale`）：**仅**在配置了手工 `ground_corner_keepout_xyr` 时，对该圆柱施加硬禁行。

**A\* 单步代价（概念）**：

```text
cost = 欧氏格距
     + ground_climb_penalty × |Δz|
     + (若在转角软环内) ground_corner_keepout_penalty
     + (若距墙 < wall_clearance) wall_clearance_weight × gap²
     + (同层意图且 z 出带) ground_leave_floor_penalty × |z−band| / res
```

若目标格落在**手工**硬核内 → **不可扩展**（`canMoveGround` 直接拒绝）。规划成功日志：`soft_hits` / `near_seed`（距自动 L 种子 <0.45 m）/ `explicit_hard`（仅手工 xyr）/ `min_seed_xy` / `min_wall_xy`。跟踪卡死时：`tracking_footprint_collision ... pose=x,y,z path_t=...`。

### `planning.ground_corner_keepout_radius`

- **类型**：`double`（米）
- **yaml 当前值**：`1.2`
- **含义**：自动检测墙体内角 L 后的影响半径。硬核半径 = `radius × hard_core_scale`（默认约 `0.6 m`）。起终点附近有豁免气泡，避免平台起点 `iterations=1`。
- **设为 `0`**：关闭自动转角检测。
- **调参**：仍贴栏杆转角 → 增大到 `1.6~1.8`；楼梯过不去 → 略减小或降低 `hard_core_scale`。

### `planning.ground_corner_keepout_hard_core_scale`

- **类型**：`double`，范围 `[0, 1]`
- **yaml 当前值**：`0.5`
- **含义**：硬禁行核相对 `radius` 的比例。`0` = 仅软代价；`1` = 整个半径硬封（易把平台围死）。
- **设计意图**：硬核挡住「贴栏杆内角」；软环仍可通行但很贵，避免起点落在软环内时 `no_path`。

### `planning.ground_corner_keepout_penalty`

- **类型**：`double`
- **yaml 当前值**：`50.0`
- **含义**：转角种子处的峰值额外代价，向外按 `(1-d/r)²` 衰减到 0。

### `planning.ground_wall_clearance`

- **类型**：`double`（米）
- **yaml 当前值**：`0.55`
- **含义**：中心线到 occupied 墙/栏杆的期望净空。小于该距离时按平方差距加代价。宜略大于狗半宽（footprint `|y|≤0.4`）。

### `planning.ground_wall_clearance_weight`

- **类型**：`double`
- **yaml 当前值**：`12.0`
- **含义**：贴墙代价权重。越大路径越靠通道/平台中央。

### `planning.ground_corner_keepout_xyr`

- **类型**：`double[]`，交错 `x, y, radius` 三元组
- **默认**：省略该键（空）；**不要**写成 `[]`
- **含义**：手工圆柱禁行/加价区（与自动检测叠加）。硬核同样按 `hard_core_scale` 缩放。

### `planning.ground_corner_keepout_z_min` / `planning.ground_corner_keepout_z_max`

- **类型**：`double`（米）
- **yaml 当前值**：`-1.0` / `8.0`
- **含义**：转角检测与贴墙偏好生效的世界高度带。

### `planning.ground_corner_keepout_height_span_min`

- **类型**：`double`（米）
- **默认**：`0.5`（代码默认；yaml 未写则用此值）
- **含义**：邻域支撑高度差 ≥ 该值才启用**自动** L 检测（区分楼梯区与平地直角墙）。

---

### 平台航点（可选，默认关闭）

跨层时可选地把全局 A* 拆成「起点 → 平台 → 终点」。**默认关闭、无需配置坐标**；日常靠转角硬核+软环 + 禁止边走边爬即可。仅当自动策略仍斜穿、且你已知中层平台世界坐标时再开。

### `planning.ground_landing_via_enabled`

- **类型**：`bool`
- **yaml 当前值**：`false`
- **含义**：是否启用平台分段规划。`false` 时忽略 via 点列表。某段 via 不可达时会回退为起终点直搜（不硬失败）。

### `planning.ground_landing_via_min_dz`

- **类型**：`double`（米）
- **yaml 当前值**：`0.7`
- **含义**：仅当 `|goal.z - start.z| ≥` 该值时才插入平台航点；同层或高度差很小的导航不绕平台。

### `planning.ground_landing_via_radius`

- **类型**：`double`（米）
- **yaml 当前值**：`0.8`
- **含义**：分段 A* 认为「到达平台」的 XY 容差；同时允许约 `max(resolution, 0.35)` m 的 z 偏差。平台点云略偏仍可判定到达。

### `planning.ground_landing_via_points`

- **类型**：`double[]`，交错 `x, y, z` 三元组
- **默认**：省略该键（空）；**不要**写 `[]`
- **含义**：启用 via 时的中间目标。按爬升方向排序后依次访问。示例：`[8.5, -2.8, 1.5]`。
- **启用步骤**：设 `ground_landing_via_enabled: true`，并写出至少一个 `x,y,z` 三元组（在 RViz 量中层平台中心）。

---

### 3D footprint 与俯仰

### `planning.ground_footprint_enabled`

- **类型**：`bool`
- **默认**：`true`（yaml）
- **含义**：启用机器狗朝向长方体 footprint（轨迹检查、跟踪停车、RViz 线框）。

### `planning.ground_footprint_xy`

- **类型**：`double[]`，交错 `x,y`（机体系，x 前 y 左）
- **默认**：`[-0.67,-0.4,-0.67,0.4,0.67,0.4,0.67,-0.4]`（约长 1.34 m，宽 0.8 m）
- **含义**：身体底面多边形顶点，挤出为高度 `ground_body_height` 的棱柱。

### `planning.ground_body_clearance`

- **类型**：`double`（米）
- **默认**：`0.5`
- **含义**：身体底面相对局部地面的离地高度（站立约 50 cm）。俯仰时箱体随楼梯斜度倾斜；踏面/楼板在 `terrain_ignore_margin` 内不计碰撞（腿支撑）。

### `planning.ground_body_height`

- **类型**：`double`（米）
- **默认**：`0.35`
- **含义**：躯干箱体厚度（底面向上）。

### `planning.ground_max_pitch_rad`

- **类型**：`double`（弧度）
- **默认**：`0.7`
- **含义**：俯仰角限幅（约 ±40°）。

### `planning.ground_terrain_ignore_margin`

- **类型**：`double`（米）
- **默认**：`0.55`（实现里还会与 `0.95 * body_clearance` 取较大值）
- **含义**：相对局部地面高度在此裕度内的 occupied 视为踏面/楼板，不与身体碰撞；侧墙/高出障碍仍会挡住。

### `planning.ground_footprint_sample_step`

- **类型**：`double`（米）
- **默认**：`0.1`
- **含义**：估计俯仰时在 footprint 前/后缘上采样地形的步长。

**Footprint 接入位置**

| 阶段 | 行为 |
|------|------|
| A* | 不用完整 footprint 盒；用球 + 支撑；转角为**硬核禁入 + 软环代价** |
| 规划后 | 沿路径采样 footprint；转角扫中间 yaw；疑似碰撞发 `plan_footprint_warning`（不直接失败） |
| 跟踪 | 当前位姿 footprint 碰撞 → `tracking_footprint_collision` 停车 |
| 俯仰 | **优先地图前后地形高差**（与楼梯坡平行；正 pitch = 抬头）；路径 Δz/Δs 仅在地图近似水平时弱混合；IMU 开启时优先 IMU |
| 可视化 | `/nav3d/footprint_marker`：规划采样（绿）+ 当前位姿（橙）；2D grid 投影不做转角代价计算（避免启动卡死） |

### `imu.topic` / `imu.use_for_body_attitude`

- **默认**：`/imu`，`false`
- **含义**：预留 IMU。`use_for_body_attitude: true` 且消息姿态有效时，优先用 IMU 的 pitch/roll；否则用地图前后支撑高差拟合楼梯斜度。

---

## 4. 规划相关（ground 仍生效）

### `planning.mode`

- **类型**：`string`，`2d` | `3d`
- **默认**：`3d`
- **ground + 3d**：邻居可含竖直 `|Δz|≤1`，可爬楼（但禁止水平+竖直同一步）。
- **ground + 2d**：不允许变 z，只能同层地面走（楼梯场景一般用 `3d`）。

### `planning.search_algorithm`

- **类型**：`string`，`astar` | `jps`
- **默认**：`astar`
- **说明**：配置会读入；ground 路径搜索当前走的是桥接内的 **ground-supported A\***（支撑 + 转角硬核/软环 + 贴墙代价 + 可选 via）。爬楼场景建议保持 `astar`。

### `planning.use_octomap_map`

- **类型**：`bool`
- **默认**：`false`
- **含义**：`false` 用稠密 `VoxelGridMap` 规划；`true` 用 OctoMap 叶子作全局规划底图。
- **ground 建议**：默认 `false` 更稳、行为更直观。

### `planning.inflation_radius`

- **类型**：`double`（米）
- **默认**：yaml 中为 `0.0`（UAV 文档常见 0.5）
- **ground 下**：**不生效**（代码在 ground 时跳过 `InflationLayer`，避免地板向上膨胀把踏面抬到断开层）。
- **UAV 下**：作为自由空间机体膨胀半径。
- **ground 替代**：使用 `planning.ground_robot_radius`、转角硬核/软环与贴墙代价。

### `planning.uav_endpoint_snap_radius_cells`

- **类型**：`int`
- **默认**：`12`
- **ground 下**：**不使用**（起终点走 ground snap）。
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

ground 爬楼 demo 通常保持动力学检查关闭；打开后需按真实机器人限速限加速度设置。

**轨迹生成**：ground 强制 `prefer_path_following_trajectory`，沿 A* 折线生成 degree-1 轨迹，**不**跑三次 B 样条优化（避免弦线斜穿楼梯井）。

---

## 5. 地图参数（决定格子与单步台阶高度）

### `map.resolution`

- **类型**：`double`（米）
- **默认**：`0.2`
- **ground 关键**：
  - 体素边长；
  - A* 单次扩展 `|Δz|≤1` → **单步最大爬升 ≈ resolution**（0.2 m → 约 20 cm）；
  - `support_*_cells`、`snap_radius_cells`、`ground_robot_radius`、转角半径换算成格数都依赖它。

### `map.pcd_path` / `map.pcd_loader`

- 点云路径与加载器（`builtin` / `pcl`）。launch 里的 `pcd_path:=...` 会覆盖空默认。

### `map.min_points_per_voxel` / `map.min_cluster_voxels`

- 预处理：格内最少点数、最小连通体素数。过大易丢薄踏面；过小噪声更多。

### `map.insert_free_space_rays` + `map.sensor_origin.*`

- 是否从传感器原点向命中点插 free 射线。ground 默认 `false`：只标 occupied，未知区在本实现里常被当可走（再加支撑约束）。

### `map.occupancy_grid_min_z` / `max_z` / `max_cells`

- 仅影响 **2D 占据栅格可视化** `/nav3d/occupied_grid` 的高度切片与上限，不直接改 3D ground 搜索规则。

---

## 6. 局部地图 `local_grid.*`

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
**说明**：3D 雷达局部重规划可缓解贴障卡住，**不能**单独修正「斜穿上楼」的全局拓扑；需配合转角软代价 / 爬升惩罚（及可选平台 via）。

---

## 7. 安全 `safety.*`（ground 下大部分前方碰撞逻辑被跳过）

| 参数 | 含义 | ground 注意 |
|------|------|-------------|
| `safety.enabled` | 是否构造安全监控 | ground 在控制循环里**直接 return**，不跑轨迹前方碰撞/急停那套 |
| `safety.emergency_stop_horizon` | 急停时间窗（秒） | UAV 有效；ground 基本用不上 |
| `safety.lookahead_horizon` | 前瞻碰撞时间窗 | 同上 |
| `safety.replan_cooldown_sec` | 重规划冷却 | 同上 |
| `safety.local_replan_*` | 局部 A* 重规划 | 主要为 UAV 自由空间场景 |

楼梯离散路径易「擦」立板，故 ground 故意不因 ahead-collision 急停；跟踪侧仍可用 footprint 硬停。

---

## 8. 控制器 `controller.*`（ground 会改写）

| 参数 | yaml 默认 | ground 行为 |
|------|-----------|-------------|
| `controller.enabled` | `true` | 是否发 `cmd_vel` |
| `controller.control_dt` | `0.1` | 控制周期（秒） |
| `controller.lookahead_time` | `0.4` | 被 **上限压到 ≤ 0.25** |
| `controller.max_speed` | `1.0` | 被 **上限压到 ≤ 0.55** |
| `controller.goal_tolerance` | `0.15` | 到点判定（米） |
| `controller.command_frame` | `map` | `map` / `body` |
| `controller.motion_model` | `uav` | 若设成 omni/diff/ackermann，ground 会 **强制改成 uav**（需要 `linear.z` 爬楼） |
| `controller.min_turning_radius` | `1.0` | 控制层转弯半径；可与 `ground_corner_keepout_radius` 同量级配合 |
| `controller.max_linear_acceleration` 等 | 见 yaml | 加速度、偏航增益/限速等 |
| `controller.block_occupied_voxels` | 未写则自动 | ground **默认 false**（楼梯弦线易穿立板，挡住会卡死） |
| `controller.command_path_sample_step` | `0.05` | 路径采样步长（米） |

---

## 9. 其它与起终点 / 轨迹显示

| 参数 | 含义 |
|------|------|
| `frame_id` | TF/消息坐标系，需与 RViz Fixed Frame 一致 |
| `start.x/y/z` | 节点初始起点（building2_9 默认楼梯平台）；点击 `/nav3d/start` 会覆盖 |
| `trajectory.publish_max_poses` | 发布轨迹最多点数 |

`nav3d_cmd_vel_pose_sim` 段为仿真位姿节点参数（`start_*`、`update_rate`、`max_step`、`cmd_timeout` 等），与 traversability 模式独立，但 ground 爬楼 demo 通常一并启动。

---

## 10. Ground 调参速查

| 目标 | 优先调 |
|------|--------|
| 一步能爬的台阶高度 | `map.resolution`（单步 ≈ 1 格） |
| 禁止一层斜穿二层 / 贴栏杆内角 | 增大 `ground_corner_keepout_radius` / `penalty`；确认 `hard_core_scale≈0.5`；仍 no_path 看起终点豁免 |
| 路径仍偏内侧 | 增大 `ground_wall_clearance` / `wall_clearance_weight` / `corner_keepout_penalty` |
| 外绕不够强 | `corner_keepout_penalty: 35~50` |
| 硬核把平台封死 / no_path | 减小 `hard_core_scale`（如 `0.4`）或 `radius` |
| 手工钉死某个转角 | `ground_corner_keepout_xyr: [x,y,r]`（可选；勿写空 `[]`） |
| 仍斜穿且已知平台坐标 | 开 `ground_landing_via_enabled` 并设 `ground_landing_via_points` |
| 更偏好平路再爬 | 增大 `ground_climb_penalty` |
| 同层起终点仍爬上楼 | 增大 `ground_leave_floor_penalty`；若日志 `leave_floor>0` 说明 L1 岛屿不连通（地图几何） |
| 同层 `no_path` 且 `same_floor_soft_z` / 旧硬带 | 确认已用软离层代价（非硬截断）；必要时减小 `leave_floor_penalty` |
| RViz 起点/目标埋进楼板 | 已自动上抬；失败看 `ground_endpoint_snap_failed` → 增大 `ground_snap_radius_cells` |
| 更贴墙/更危险 | 减小 `ground_robot_radius` |
| 更保守离障 | 增大 `wall_clearance` / `corner_keepout_penalty`（**勿**把 `robot_radius` 加到 ≥0.3） |
| 点击吸附更远/更近 | `ground_snap_radius_cells` |
| 踏面略偏仍算有地 | 增大 `ground_support_xy_radius_cells` 或保持 strict=false |
| 更严“正下方才算地” | `ground_strict_direct_support: true` |
| 身体俯仰不对齐楼梯 | 查 footprint 前后采样 / `max_pitch_rad`；确认已重启新二进制 |
| 爬楼速度/平滑 | `controller.max_speed` / `lookahead_time`（仍受 ground 上限夹紧） |
| 瞬间 `no_path`（iterations≈2） | 多为起终点不可走或硬核过大；减小 `hard_core_scale` / `radius` |

---

## 11. 与 UAV 对照（同文件参数）

| 能力 | ground | uav |
|------|--------|-----|
| 脚下支撑 | 要 | 不要 |
| `inflation_radius` | 忽略 | 生效 |
| 起终点吸附 | `ground_snap_*` + support | `uav_endpoint_snap_radius_cells` |
| 转角硬核+软环 / 平台 via | 生效（via 默认关） | 不用 |
| 轨迹优化 | 路径跟随（无 B 样条平滑） | 三次 B 样条 + rebound |
| 安全 ahead 碰撞 | 跳过 | 启用 |
| 控制器 | 限速 + 强制可出 vz | 按配置运动模型 |

---

## 12. yaml 片段（与当前默认对齐）

```yaml
planning.ground_strict_direct_support: false
planning.ground_snap_radius_cells: 6
planning.ground_support_xy_radius_cells: 1
planning.ground_support_depth_cells: 1
planning.ground_robot_radius: 0.25
planning.ground_footprint_enabled: true
planning.ground_footprint_xy: [-0.67, -0.4, -0.67, 0.4, 0.67, 0.4, 0.67, -0.4]
planning.ground_body_clearance: 0.5
planning.ground_body_height: 0.35
planning.ground_max_pitch_rad: 0.7
planning.ground_terrain_ignore_margin: 0.55
planning.ground_footprint_sample_step: 0.1
# 转角硬核 + 软环（无需坐标）
planning.ground_corner_keepout_radius: 1.2
# planning.ground_corner_keepout_xyr: [9.0, -3.0, 1.0]  # 可选；勿写空 []
planning.ground_corner_keepout_z_min: -1.0
planning.ground_corner_keepout_z_max: 8.0
# planning.ground_corner_keepout_height_span_min: 0.5
planning.ground_corner_keepout_penalty: 35.0
planning.ground_corner_keepout_hard_core_scale: 0.5
planning.ground_wall_clearance: 0.55
planning.ground_wall_clearance_weight: 15.0
planning.ground_climb_penalty: 0.85
planning.ground_leave_floor_penalty: 25.0
# 平台 via（默认关）
planning.ground_landing_via_enabled: false
planning.ground_landing_via_min_dz: 0.7
planning.ground_landing_via_radius: 0.8
# planning.ground_landing_via_points: [8.5, -2.8, 1.5]
imu.topic: /imu
imu.use_for_body_attitude: false
```

### 当前 yaml 关键值速查

| 参数 | 当前值 | 一句话 |
|------|--------|--------|
| `ground_robot_radius` | `0.25` m | A* 球净空（≥0.3 会掐断 building2_9） |
| `ground_corner_keepout_radius` | `1.2` m | 转角影响半径 |
| `ground_corner_keepout_hard_core_scale` | `0.5` | 仅对手工 xyr 硬封；自动 L 只用软环 |
| `ground_corner_keepout_penalty` | `35.0` | 软环内每格额外代价 |
| `ground_wall_clearance` | `0.55` m | 贴墙开始加价的距离 |
| `ground_wall_clearance_weight` | `15.0` | 贴墙代价权重 |
| `ground_climb_penalty` | `0.85` | 每格 `|Δz|` 爬升附加代价 |
| `ground_leave_floor_penalty` | `25.0` | 同层意图时离层软代价（非硬截断） |
| `ground_landing_via_enabled` | `false` | 默认不用平台坐标 via |
| `ground_snap_radius_cells` | `6` | RViz z=0 吸附搜索半径（格） |
