# 分步启动定位和导航

GPSR 自然语言导航的航点标定、配置、调用和错误处理见下文
[GPSR 规划器接入](#gpsr-navigation)。

先退出旧 quickstart、旧 Nav2 和重复的雷达/底盘节点。
此前的 standalone quickstart 发布 `map -> base_link`，不能与底盘的
`odom -> base_link` 一同充当导航 TF；请改用本包的定位入口。

终端 1（若底盘已运行，可跳过）：

```bash
sudo ip link set can0 up type can bitrate 500000
ros2 launch tracer_navigation base.launch.py
```

终端 2（若已经是本机 PointCloud2 驱动，可跳过）：

```bash
ros2 launch tracer_navigation mid360.launch.py
```

终端 3：

```bash
ros2 launch tracer_navigation localization.launch.py
```

定位会使用 `map_corridor.pcd` 与配套的 `nav2/map_corridor.yaml` 尝试自动初始化。
如果无法确认位置，使用 RViz 的 **2D Pose Estimate**。确认点云与地图重合后，
终端 4 启动导航，使用 **Nav2 Goal** 发送目标：

```bash
ros2 launch tracer_navigation nav2.launch.py
```


默认开启局部体素发布。`nav2.launch.py` 自动启动
`/local_costmap/voxel_visualizer`，订阅 `/local_costmap/voxel_grid`
并转换为 `/local_costmap/voxel_marked_cloud`。RViz 的
`Controller → Local VoxelGrid` 默认启用，以 0.1 m 方块按高度着色显示占据体素，
无需额外运行转换命令；观察高度时可将 Views 切换为 Orbit。
分步启动定位和 Nav2 时也使用同一显示链路；仅启动定位时须等 Nav2 发布体素。
自定义 Nav2 配置若关闭 `voxel_layer` 或 `publish_voxel_map`，则不启动转换节点。


# 本机 Tracer / MID360 定位与导航

日常启动见[导航启动命令速查](../docs/navigation_startup.zh-CN.md)，包含命令与模块对应、主要 topic 的发布/订阅关系。

完整工程说明见 [开发手册](../docs/real_robot_simulation_engineering_notes.zh-CN.md)，
含顺序启动、模块参数、topic/TF、算法与标定笔记；引用资料见
[分类参考清单](../docs/references.zh-CN.md)。本页保留功能包快速操作与验证说明。

ROS 2 Humble 的硬件适配集中在本包。依赖原始 `tracer_base`、
`livox_ros_driver2`、`fast_lio`、`lidar_localization_ros2` 与 Nav2，算法源码仍在
各自工作区。地图数据保存在 `~/maps`，不复制进功能包。

临时改动与各项目源码、安装目录的对应关系见
[同步与原入口兼容记录](docs/synchronization.md)。本包运行使用 `/home/steno` 下的
项目文件，`/tmp` 用于运行时生成参数和测试输出。

原有 `msg_MID360_launch.py`（CustomMsg）、`rviz_MID360_launch.py`（PointCloud2）
和 FAST-LIO `mapping.launch.py` 保留原样。新增 `pc2_MID360_launch.py` 为独立文件，
没有覆盖原入口。本包建图入口选择 CustomMsg，定位/导航入口选择 PointCloud2。

## 已修复的问题

2026-10-22 12:47 的 Nav2 日志直接报错：
`nav2_navfn_planner::NavfnPlanner` 不在已注册插件列表中。
本机 Humble 使用 `nav2_navfn_planner/NavfnPlanner`；行为插件也使用
`nav2_behaviors/Spin` 等带 `/` 的 ID。新的 Nav2 YAML 使用已安装插件定义，
并补齐 Humble 的 BT 插件库列表。

原导航入口默认 `/velodyne_points`，而本机为 `/livox/lidar`。
定位与导航的两个点云输入现在共用硬件配置。Nav2 入口只启动导航和 map_server，
不会再次启动定位；组合入口只创建一个定位节点。

2026-10-28 修复了定位启动约 3 秒后报
`launch configuration 'run_bringup_check' does not exist` 并关闭整组节点的问题。
本机 `lidar_localization_ros2/launch/quickstart.launch.py` 的延迟检查现在提前解析
所需参数，并在注册定时器时判断是否启用；兼容本包的分组作用域和 topic 隔离。
复核记录见[同步与兼容记录](docs/synchronization.md#2026-10-28-定位启动退出修复)。

## 文件入口

| 文件 | 用途 |
| --- | --- |
| `config/hardware.yaml` | 话题、CAN、坐标系、安装外参、机器人轮廓、地图路径 |
| `config/livox_mid360.json` | 本机雷达 IP/接收地址与 UDP 端口 |
| `config/fastlio_mid360.yaml` | FAST-LIO 建图参数；CustomMsg 输入 |
| `config/localization.yaml` | 三维地图 NDT 参数；轮式 TF 预测 |
| `config/nav2_humble.yaml` | Humble 插件、规划控制、PointCloud2 障碍物层 |
| `config/cyclonedds.xml` | 多进程发现容量，避免 Humble Cyclone DDS participant 索引耗尽 |
| `rviz/navigation.rviz` | 三维地图、点云、Nav2 地图、路径和 Goal 工具 |
| `launch/mid360.launch.py` | 雷达驱动，默认 PointCloud2；`xfer_format:=1` 为 CustomMsg |
| `launch/base.launch.py` | CAN 底盘、真实 `/odom`、`odom -> base_link` |
| `launch/mapping.launch.py` | MID360 CustomMsg + FAST-LIO 建图 |
| `launch/localization.launch.py` | Nav2 TF 模式定位、自动初始化、安装 TF、RViz |
| `launch/nav2.launch.py` | 仅导航；复用已经启动的底盘、雷达和本包定位 |
| `launch/navigation.launch.py` | 底盘 + 雷达 + 定位 + Nav2 组合入口 |

## 构建与环境

在未加载其他机器人工作区的终端中构建本包（底盘依赖已构建）：

```bash
source ~/agilex_tracer_ros2_ws/src/tracer_navigation/scripts/setup.bash
cd ~/agilex_tracer_ros2_ws
colcon build --symlink-install --packages-select tracer_navigation
```

之后在每个终端中统一加载：

```bash
source ~/agilex_tracer_ros2_ws/src/tracer_navigation/scripts/setup.bash
```

此入口依次加载 ROS、Livox、FAST-LIO、定位和本工作区；各工作区使用
`install/local_setup.bash`，避免 `install/setup.bash` 回放构建时记录的旧工作区。
定位脚本通过临时指定 `LIDAR_LOCALIZATION_OVERLAY` 保留本地库环境，同时跳过
定位工作区的历史加载链。无需重新创建已删除的 `husarion_rosbot_ws`、
`wpr_ros2_ws` 或向其他项目添加占位 setup 文件。

`--symlink-install` 使本包配置和启动文件的修改可直接生效。
新增文件、Python 入口或包依赖后重新构建。本包不配置网卡或自动启用 CAN；
继续使用已验证的硬件网络/CAN 初始化方式。

本机使用 Cyclone DDS。环境脚本在没有自定义 `CYCLONEDDS_URI` 时加载本包配置，
把自动 participant 最大索引从本机默认 9 增至 64，避免多进程导航启动失败。
如果已有自定义 DDS 配置，应把对应设置合并进去，并让所有终端使用相同配置。

底盘、雷达已经运行时：

```bash
ros2 launch tracer_navigation navigation.launch.py start_base:=false start_lidar:=false
```

若本包定位也已运行，使用 `nav2.launch.py`；也可以设置
`start_localization:=false`。不要将旧 standalone 定位当作这里要求的定位模式。

覆盖地图路径、硬件配置和显示设置：

```bash
ros2 launch tracer_navigation navigation.launch.py \
  map:=/home/steno/maps/map_corridor.pcd \
  map_yaml:=/home/steno/maps/nav2/map_corridor.yaml \
  hardware_config:=/home/steno/agilex_tracer_ros2_ws/src/tracer_navigation/config/hardware.yaml \
  rviz:=true
```

自动初始化可设 `auto_initialize:=false`，改用手动初始位姿。
仅做启动配置检查可设 `autostart:=false`；查看全部入口参数可加 `--show-args`。

## 导航障碍物高度过滤

已确认地面在 `base_link` 中的 Z 为 0，机器人顶部为 Z=1.50 m。
`nav2.launch.py` 自动启动 `obstacle_height_filter`，按每帧点云时间戳查询
雷达到 `base_link` 的 TF，仅保留 `0.08 <= Z(base_link) <= 1.50 m` 的点。
配置位于 `config/hardware.yaml` 的 `obstacle_filter.min_height/max_height`；
0.08 m 是地面排除余量，低于该高度的矮障碍也会被排除，可按实际需求调小。
这属于已知地面高度下的裁剪，不是自动地面平面分割。

原始 `/livox/lidar` 继续用于定位与建图；过滤后发布到
`/navigation/obstacle_cloud`，仅供局部 VoxelLayer 和全局 ObstacleLayer 使用。
保留原始点云的坐标系、时间戳和字段，确保 Nav2 仍从雷达位置进行射线清除。
Nav2 YAML 中的宽高度范围是 map/odom 坐标下的二次保护，实际物理阈值只在
上述 `base_link` 过滤器中设置。不要把 1.50 m 当成 map 或雷达的绝对 Z 上限。

新增 Python 入口后，在工作区根目录重新构建，再重启原定位导航命令：

```bash
cd ~/agilex_tracer_ros2_ws
colcon build --symlink-install --packages-select tracer_navigation
source src/tracer_navigation/scripts/setup.bash
ros2 launch tracer_navigation navigation.launch.py
```

RViz 的 `Controller → Local VoxelGrid` 显示过滤后的占据体素；`MID360`
仍显示完整原始扫描。可另外添加 PointCloud2 `/navigation/obstacle_cloud`
对照过滤效果。重启 Nav2 会重建体素与代价地图，避免沿用旧天花板/地面标记。

## TF 与速度链路

```text
map --[lidar_localization]--> odom --[tracer_base]--> base_link
                                                        |
                    [安装标定 x=.252 y=0 z=.236, r=p=y=0]  |
                                                        v
                                                   livox_frame

controller_server / behavior_server -> /cmd_vel_nav
velocity_smoother                   -> /cmd_vel -> tracer_base -> CAN
```

`odom -> base_link` 来自底盘反馈速度的积分。定位根据三维点云匹配结果计算
`map -> odom`，并用已有轮式 TF 预测运动。没有发布静态/命令积分的假里程计。
`/livox/lidar` 是 PointCloud2，`/livox/imu` 是 Imu，二者的 header 都是
`livox_frame`，因此只发布一条安装 TF。IMU 预积分延续先前默认关闭设置。

安装外参采用用户提供的标定结果，未把 FAST-LIO 的 LiDAR–IMU 内部外参重复用于
车体安装 TF。`global_sensor_height_m` 是全局搜索所用的离地高度；若 base_link
不在地面，应在硬件配置中加上相应高度。

机器人尺寸采用用户提供的长 610 mm（X）、宽 702 mm（Y）。假设 base_link 位于
车体矩形的 XY 中心，footprint 为 `x=±0.305 m、y=±0.351 m`，不再采用旧的
0.7 m 圆形半径。若 base_link 原点偏离中心，应相应平移这四个顶点。
Nav2 YAML 的初始速度上限为 0.3 m/s、0.6 rad/s。
未知栅格保持未知，默认不在未知区域规划。

三维地图和二维地图应保持同一 XY 坐标系；直接投影得到的配套地图满足这一点，
另行建图得到的二维地图需要先配准。

## FAST-LIO 建图

退出地图定位、导航和现有雷达驱动后：

```bash
ros2 launch tracer_navigation mapping.launch.py
```

该入口使用 `xfer_format=1` 和 `preprocess.lidar_type=1`，保留逐点时间。
需要同时启动底盘可加 `start_base:=true`；雷达已运行 CustomMsg 驱动可加
`start_lidar:=false`。`map_output` 默认 `~/maps/fastlio_mapping.pcd`，可覆盖。

```bash
ros2 service call /map_save std_srvs/srv/Trigger '{}'
```

`/map_save` 使用指定的 `map_output`。FAST-LIO 退出时的扫描缓存保存仍由其原始
代码写到编译期 `ROOT_DIR/PCD/scans.pcd`；这个路径不受 `map_output` 控制。
FAST-LIO 的 `camera_init -> body` 是其建图坐标系，未把它硬拼为导航的轮式 odom。

## 诊断和验证

```bash
ros2 run tracer_navigation doctor --navigation
ros2 node info /lidar_localization
ros2 topic info /livox/lidar -v
ros2 topic echo /odom --once --field header
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo map odom
ros2 lifecycle get /planner_server
ros2 action list -t
```

`doctor` 只读取消息与 TF，不发速度命令。定位尚未初始化时 `map -> odom`
可能暂不可用。使用 `python3 -m pytest test/` 运行配置和启动生命周期测试；隔离环境的
导航验证脚本见 `test/smoke_navigation.py`，其模拟底盘不会连接 CAN。

```bash
cd ~/agilex_tracer_ros2_ws/src/tracer_navigation
python3 -m pytest -q test/test_configuration.py test/test_launch_lifecycle.py
python3 test/smoke_navigation.py
python3 test/smoke_obstacle_filter.py
python3 test/smoke_localization.py
```

两个 smoke 脚本分别强制使用 localhost ROS Domain 173/174，避免向生产域发布测试
消息。第二个脚本读取现有 PCD，生成与已知位姿对应的模拟扫描，并通过组合入口
验证 NDT 匹配和 `map -> odom` 计算；要求本机地图文件存在，不代表实车精度测试。

2026-10-22 验证结果：配置契约测试 7 项通过；导航测试的 8 个生命周期服务均 active，
ComputePathToPose 与 NavigateToPose 成功，平滑速度驱动模拟底盘到达目标容差。
PCD 模拟扫描测试中 NDT 接受匹配，标定安装 TF 正确，`map -> odom` 与已知
两组位姿的解析关系一致，实际地图下的 8 个 Nav2 服务也全部 active。
检查时真实雷达网口未连接、没有 can0，因此未执行实车运动或定位精度验收。

2026-10-28 复核：配置与启动生命周期测试 11 项通过；定位依赖包相关测试 88 项通过。
修复前新增回归测试复现了延迟参数缺失，修复后分步定位和组合入口均通过。
PCD 模拟扫描获得 55 次健康匹配、8 个 Nav2 服务全部 active，激活后继续运行
4 秒以检查延迟退出；仍不代表实车定位精度验收。

配置来源：本机既有 Livox/FAST-LIO 参数、lidar_localization_ros2 的
PointCloud2 Nav2 模板，以及 `/opt/ros/humble/share/nav2_bringup` 的 BT 库列表
和 RViz 模板；各上游文件仍保留在原工作区。

<a id="gpsr-navigation"></a>

## GPSR 规划器接入：航点、参数与命令

调用与动作调度代码位于独立工作空间 `~/gpsr_ros2_test`，以 ROS 2 Humble 和
`colcon build --symlink-install` 构建。本包继续负责底盘、雷达、定位和 Nav2；
GPSR 新入口复用这些已经运行的节点，不会自动启动或激活它们。
规划器工作空间的说明见 [GPSR README](../../../gpsr_ros2_test/README.md)。

```text
自然语言文本
  → /baidu_llm_service（节点 /llm_service_node）
  → 动作列表字符串
  → /gpsr/execute_actions（节点 /gpsr_navigation_executor）
  → 检查整条动作链、航点和导航依赖
  → /navigate_to_pose（nav2_msgs/action/NavigateToPose）
  → controller_server / behavior_server → /cmd_vel_nav
  → velocity_smoother → /cmd_vel → /tracer → CAN → 底盘
```

原 GPSR 的地点调度与顺序调用逻辑移植到 `seu_llm`，原 ROS 1 `move_base`
调用改为 Nav2 Action。运行时不依赖 ROS 1 的 actionlib 或 SMACH。

### 支持的动作与执行边界

| 动作列表元素 | 当前行为 |
| --- | --- |
| `['move_to', 'kitchen']` | 导航到名称或别名对应的地图位姿 |
| `['lead_to', 'living_room']` | 导航到引导终点；不检测人员、不确认人员跟随、不播报 |
| `['leave', 'exit']` | 导航到指定离开航点 |
| `['leave']` | 导航到 `leave_waypoint` 指定的默认出口 |

仅执行 `move_to`、`lead_to`、`leave`。找人、找物、人员跟随、抓取、递交、
放置、说话、问答等动作，以及所有未知动作，一律拒绝。
动作链必须包含 1～32 项；动作名称区分大小写。`move_to` 和 `lead_to`
必须带一个非空地点字符串，`leave` 可省略地点或带一个地点字符串。

执行器在发送第一个 Nav2 目标前，校验**整条链**的动作、参数、航点、坐标系和依赖。
例如 `[['move_to', 'kitchen'], ['grasp', 'cup']]` 返回
`fail: not supported action: grasp (index 1)`，不会先去厨房。
后续航点未知或未标定，也不会先执行前面的移动。
LLM 保留原始任务中的其他动作，执行器不会静默删除它们。

导航按顺序执行，仅在当前目标返回 `STATUS_SUCCEEDED` 后发送下一项。
运行中发生失败、取消、超时或依赖失效时终止剩余动作；已经完成的移动不会回滚。

### 配置文件位置与修改生效方式

| 配置/源码 | 修改内容 | 生效方式 |
| --- | --- | --- |
| `~/gpsr_ros2_test/config/waypoints.yaml` | 地点、别名、航点坐标、默认出口；链接到 `src/seu_llm/config/waypoints.yaml` | 每次规划和每条动作链执行前重新读取；修改航点无需重新构建或重启 |
| `~/gpsr_ros2_test/src/seu_llm/config/navigation.yaml` | 执行器使用的节点、话题、TF、Action 名称与超时 | symlink 安装下修改源文件后重启执行器；诊断命令下次启动时读取 |
| `~/gpsr_ros2_test/config/qianfan.env` | `QIANFAN_AUTHORIZATION` API 凭据 | 重新加载环境并重启 LLM 服务；不要把密钥写入 README 或提交到版本库 |
| `~/gpsr_ros2_test/src/seu_llm/seu_llm/planner.py` | `MODEL` 模型标识、`API_URL` 接口地址 | 修改后重启 LLM 服务 |
| 本包 `config/hardware.yaml` | 实际 CAN、话题、坐标系、安装外参、地图路径和车体尺寸 | 按本包启动方式重启相关机器人节点 |
| 本包 `config/localization.yaml` | NDT 定位参数 | 重启定位节点 |
| 本包 `config/nav2_humble.yaml` | Nav2 规划控制、避障、速度上限和到达容差 | 重启 Nav2；GPSR 参数不改变这些控制参数 |

当前模型为 `ernie-5.5-flash-preview`，仍通过 requests 调用
`POST https://qianfan.baidubce.com/v2/chat/completions`，保留 Bearer、
`model/messages` 请求方式及原有动作列表字符串输出。替换同平台模型时修改
`planner.py` 中的 `MODEL`。原 40 条任务示例保留，提示词附加当前航点名称、
别名与 `leave` 用法。

LLM 服务还提供 ROS 参数：`plan_set_path`（默认包内
`plan_sets/plan_set_2025.json`）、`request_timeout=30.0`（每次 HTTP 请求超时，秒）、
`max_attempts=5`（最多尝试次数）、`waypoints_file`（航点文件）。这些参数在服务启动时读取，
修改后重启服务。

### 航点文件格式与地点名称

默认文件 `~/gpsr_ros2_test/config/waypoints.yaml` 初始内容如下：

```yaml
frame_id: map
leave_waypoint: exit
waypoints:
  initial_location:
    aliases: [initial location, home, 起点]
    pose: null
  kitchen:
    aliases: [厨房]
    pose: null
  living_room:
    aliases: [living room, 客厅]
    pose: null
  exit:
    aliases: [出口]
    pose: null
```

| 字段 | 含义与要求 |
| --- | --- |
| `frame_id` | 航点坐标系，默认 `map`；必须与执行器 `map_frame` 及当前 Nav2 地图一致 |
| `leave_waypoint` | 无地点参数的 `['leave']` 使用的名称或别名，默认 `exit`；对应航点也必须标定 |
| `waypoints.<name>` | 规划器输出的地点名称，可自由新增，例如 `bedroom` |
| `aliases` | 同一地点的自然语言别名；中文名称需要显式配置 |
| `pose: null` | 尚未标定；引用此航点时拒绝执行 |
| `pose.x`、`pose.y` | 当前地图中的平面坐标，单位米，必须为有限数值 |
| `pose.yaw` | 终点朝向，单位弧度，必须为有限数值；0 朝地图 +X，正方向绕 +Z 逆时针 |

名称匹配忽略英文大小写、把下划线视为空格并合并连续空白，例如
`living_room` 和 `living room` 可匹配同一项；不同航点不能占用相同名称或别名。
`initial_location` 只是一个需要标定的固定地点，不会自动记为本次启动位置。
所有初始坐标均为空，必须按实际地图标定后使用。

### 手动标定航点

先启动底盘、PointCloud2 雷达和定位，确认点云与地图重合，且
`map → odom → base_link` 正常。标定时固定使用与导航相同的地图。
本机三维定位地图为 `~/maps/map_corridor.pcd`，二维导航地图为
`~/maps/nav2/map_corridor.yaml`，路径由本包 `config/hardware.yaml` 指定；
两者必须处于同一 XY 坐标系，更换或重新配准地图后应重新标定航点。

每个参与实车导航的终端先加载：

```bash
source ~/gpsr_ros2_test/scripts/setup_robot_env.bash
```

方法一：手动把机器人放到目标地点和朝向，确认定位稳定后采集当前位姿。例如，
机器人已经位于厨房目标点时执行：

```bash
ros2 run seu_llm waypoint capture kitchen --aliases 厨房 'cooking area'
```

随后将机器人移到出口目标点并调整朝向，再采集出口：

```bash
ros2 run seu_llm waypoint capture exit --aliases 出口
ros2 run seu_llm waypoint list
```

`capture` 读取航点 `frame_id → base_link` 的最新 TF，默认等待 5 秒，要求
TF 时间有效且最近 3 秒内更新；可用 `--timeout 10.0` 调整等待时间。
该工具固定采集 `base_link`，修改执行器 `base_frame` 不会自动修改采集工具。

方法二：从地图量取目标位置与朝向，用 `set` 写入。**以下数值仅说明格式，
必须替换为实际标定值**：

```bash
ros2 run seu_llm waypoint set kitchen --x 1.2 --y -0.4 --yaw 0.0 --aliases 厨房 'cooking area'
```

保存后的 `pose` 形式为 `pose: {x: 1.2, y: -0.4, yaw: 0.0}`。
工具会校验数值、名称和别名，校验通过后原子保存文件；省略 `--aliases`
保留该地点原有别名，提供时替换原有别名。`list`、`set`、`capture` 均不会发出移动命令。
RViz 的 **2D Pose Estimate** 用于初始化定位，不是保存航点；**Nav2 Goal** 会发起导航。

指定其他航点文件时，`--file` 是全局选项，必须放在子命令之前：

```bash
ros2 run seu_llm waypoint --file /absolute/path/waypoints.yaml capture exit --aliases 出口
```

通过环境变量 `GPSR_WAYPOINTS_FILE` 可统一改变工具和节点的默认文件；也可以启动时
显式传入，并保证 LLM 与执行器读取同一份文件：

```bash
ros2 launch seu_llm navigation_planner.launch.py waypoints_file:=/absolute/path/waypoints.yaml
```

航点文件在新请求时读取，已经接收的动作链使用接收时解析出的坐标；执行期间修改
文件不会让当前链的目标突然改变。

### 导航执行器参数

执行器从 `~/gpsr_ros2_test/src/seu_llm/config/navigation.yaml` 的
`gpsr_navigation_executor.ros__parameters` 读取默认参数，ROS 命令行参数可覆盖。

| 参数 | 默认值 | 用途 |
| --- | --- | --- |
| `map_frame` / `odom_frame` | `map` / `odom` | 航点地图坐标系与轮式里程计坐标系 |
| `base_frame` / `lidar_frame` | `base_link` / `livox_frame` | 车体与雷达坐标系 |
| `navigate_action` | `/navigate_to_pose` | Nav2 目标导航 Action |
| `execute_action` | `/gpsr/execute_actions` | 本执行器提供的动作链 Action |
| `base_node` | `/tracer` | 当前底盘实际 ROS 节点名；不是可执行文件名 `tracer_base_node` |
| `cloud_node` | `/livox_lidar_publisher` | 点云发布节点 |
| `localization_node` | `/lidar_localization` | 定位节点 |
| `odom_topic` | `/odom` | `nav_msgs/msg/Odometry`，应为 `odom → base_link` |
| `cloud_topic` | `/livox/lidar` | `sensor_msgs/msg/PointCloud2`，应在 `livox_frame` |
| `pose_topic` | `/localization/pose_with_covariance` | `geometry_msgs/msg/PoseWithCovarianceStamped`，应在 `map` |
| `alignment_topic` | `/alignment_status` | `diagnostic_msgs/msg/DiagnosticArray`，用于检查 NDT 匹配健康状态 |
| `map_topic` | `/map` | `nav_msgs/msg/OccupancyGrid`，应在 `map` |
| `cmd_vel_nav_topic` / `cmd_vel_topic` | `/cmd_vel_nav` / `/cmd_vel` | 检查控制器、速度平滑器与底盘的速度发布订阅链 |
| `waypoints_file` | 环境变量 `GPSR_WAYPOINTS_FILE`，未设置时为包内 `config/waypoints.yaml` | 此参数由执行器单独声明；实车环境脚本默认指向工作空间根目录下的配置链接 |
| `dependency_timeout` | `4.0` 秒 | 每次准备执行时等待导航依赖就绪的最长时间 |
| `data_timeout` | `3.0` 秒 | 动态消息、动态 TF、生命周期响应的新鲜度上限 |
| `goal_timeout` | `180.0` 秒 | 单个航点的导航执行超时 |
| `goal_response_timeout` | `5.0` 秒 | 等待 Nav2 接受/拒绝目标的超时 |
| `cancel_timeout` | `5.0` 秒 | 等待当前 Nav2 目标取消并终止的超时 |
| `health_poll_period` | `0.5` 秒 | 导航执行过程中的健康检查间隔 |

`lifecycle_nodes` 默认要求以下节点均为 `active`：

```yaml
lifecycle_nodes:
  - /lidar_localization
  - /map_server
  - /planner_server
  - /controller_server
  - /smoother_server
  - /behavior_server
  - /bt_navigator
  - /velocity_smoother
```

当前目标接口不依赖 `/waypoint_follower`；动作链由 GPSR 执行器逐个发送。
所有超时与检查间隔参数必须大于零。执行器启动后缓存导航参数，不能依靠
`ros2 param set` 热更新这些配置；修改后重启执行器。
例如，需要单独启动执行器并调整单航点超时时：

```bash
ros2 run seu_llm navigation_executor --ros-args -p goal_timeout:=240.0
```

不要与组合 launch 已启动的执行器重复运行。当前健康检查的部分 Nav2 角色仍使用
默认节点名，例如 `/controller_server`、`/velocity_smoother`、`/bt_navigator`、
`/map_server`；更换整组命名空间需要同时适配检查逻辑及 ROS 名称映射，
不能只修改一个参数。客户端默认连接 `/gpsr/execute_actions`，修改执行器接口名时
也必须相应重映射客户端。

### 启动顺序与命令发布

1. 按本文开头分步启动底盘、PointCloud2 雷达、定位、Nav2；已经运行的模块直接复用。
   所有终端使用一致的 ROS 域、DDS 与网络设置。确认定位后标定指令会使用的全部航点。
2. 新终端加载实车环境，启动 LLM 服务与执行器：

   ```bash
   source ~/gpsr_ros2_test/scripts/setup_robot_env.bash
   ros2 launch seu_llm navigation_planner.launch.py
   ```

   该 launch 仅启动 `/llm_service_node` 和 `/gpsr_navigation_executor`。
   也可分别运行 `ros2 run seu_llm baidu_llm_service` 与
   `ros2 run seu_llm navigation_executor`，同一节点只启动一次。
3. 客户端终端先加载同样环境，再只读检查现有导航栈：

   ```bash
   source ~/gpsr_ros2_test/scripts/setup_robot_env.bash
   ros2 run seu_llm check_navigation
   ```

   该命令输出 `state/errormsg/problems` JSON，不发目标、不激活节点，也不检查云端
   API 是否可用。LLM 与动作链执行入口的可用性由相应客户端调用检查。
4. 仅解析文本并查看动作链，不发导航目标：

   ```bash
   ros2 run seu_llm analyze_command '先去厨房，然后去客厅，最后离开'
   ```

5. 需要实际执行时运行以下命令。**这会通过 Nav2 驱动底盘移动**：

   ```bash
   ros2 run seu_llm execute_command '先去厨房，然后去客厅，最后离开'
   ```

   等价入口为 `ros2 run seu_llm analyze_command '指令' --execute`。
   输出先给出 `action_chain`，再实时给出导航反馈，最后给出
   `state/errormsg/answer` 与 `execution_results`。

实车环境脚本默认 `ROS_DOMAIN_ID=0`、`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`、
`ROS_LOCALHOST_ONLY=0`，并加载机器人和 GPSR 工作空间、API 凭据。
如果现有机器人使用其他设置，在每个终端 source 前统一设置：

```bash
export GPSR_ROBOT_ROS_DOMAIN_ID=0
export GPSR_ROBOT_RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export GPSR_ROBOT_LOCALHOST_ONLY=0
source ~/gpsr_ros2_test/scripts/setup_robot_env.bash
```

`~/gpsr_ros2_test/scripts/setup_env.bash` 默认是域 86、localhost、Fast DDS 的
隔离解析环境；与默认实车域不互通。不要在该环境下依据“节点不可用”直接判断
实车进程没有启动。两种 setup 脚本均只加载环境，不启动机器人节点。

### ROS 接口与输出格式

| 接口 | 类型与关键字段 |
| --- | --- |
| `/baidu_llm_service` | `seu_llm_interfaces/srv/SeuLlm`：请求 `task='analyze'`、`command`；响应 `state`、`errormsg`、`answer` |
| `/gpsr/execute_actions` | `seu_llm_interfaces/action/ExecuteActions`：目标 `actions`；结果 `state`、`errormsg`、`results`；反馈 `action_index/action/destination/status/distance_remaining` |
| `/navigate_to_pose` | `nav2_msgs/action/NavigateToPose`：目标 `pose`；使用 Nav2 状态和剩余距离反馈 |

`answer` 和执行目标 `actions` 都是 **Python 嵌套列表的字符串**，例如
`"[['move_to', 'kitchen'], ['lead_to', 'living_room'], ['leave']]"`。
保留原 LLM 服务的 `state/errormsg/answer` 格式；解析成功通常返回
`state: true`、`errormsg: success`。解析成功只代表得到动作链，
不代表动作受支持、航点已标定或导航执行成功。

执行器把每个航点转为 `PoseStamped`：`header.frame_id=map`（或配置的地图帧）、
时间戳为当前 ROS 时间、`position=(x,y,0)`、
`orientation=(0,0,sin(yaw/2),cos(yaw/2))`，通过 Nav2 Action 发送。
它不依靠发布 `/goal_pose` 话题，也不自行发布速度。
底盘移动仍由本包既有的 Nav2、速度平滑器和 `/tracer` 完成。

调试时可绕过 LLM，直接提交一条动作链；以下命令也会实际移动：

```bash
ros2 action send_goal /gpsr/execute_actions seu_llm_interfaces/action/ExecuteActions \
  "{actions: \"[['move_to', 'kitchen']]\"}" --feedback
```

该入口仍执行整条链和导航依赖检查，但不经过 `analyze_command/execute_command`
客户端，因此不自动生成它们的逐次命令日志。
执行结果 `results` 是 JSON 数组字符串，每项包含 `index/action/destination/status`，
失败项还包含 `error`。索引从 0 开始。

客户端常用选项：

| 选项 | 默认值 | 含义 |
| --- | --- | --- |
| `--timeout` | `180.0` 秒 | 等待 LLM 服务响应的总时间，与单次 HTTP 超时不同 |
| `--server-timeout` | `10.0` 秒 | 等待 LLM 服务、执行 Action 出现以及执行目标应答的时间 |
| `--navigation-timeout` | `1800.0` 秒 | 等待整条动作链完成的总时间；超时后请求取消 |
| `--log-dir` | 默认 `~/gpsr_ros2_test/log/commands` | 单次命令 JSON 日志目录 |

### 导航依赖检查与错误返回

执行前要求底盘、雷达、定位和所需 Nav2 节点可发现，所需生命周期节点为 active；
要求里程计、非空 PointCloud2、定位位姿、NDT 诊断持续更新，发布者、消息类型和
frame 与配置匹配。`/alignment_status` 中
`lidar_localization_ros2/alignment` 必须为 OK。
`/map` 必须收到非空 OccupancyGrid；静态地图不要求持续重发。
三维 `/initial_map` 点云不能替代 Nav2 的 `/map`。

还检查新鲜的 `map → odom`、`odom → base_link`、存在的
`base_link → livox_frame` 安装 TF，`/navigate_to_pose` 可用，以及
`/cmd_vel_nav → velocity_smoother → /cmd_vel → /tracer` 的发布订阅关系。
TF 发布者应保持唯一：定位负责 `map → odom`，底盘负责 `odom → base_link`，
静态安装变换负责 `base_link → livox_frame`。
每个新动作重新检查生命周期状态，导航过程中按 `health_poll_period` 持续检查；
异常会请求取消当前目标并停止后续动作。

失败时 `state=false`，`errormsg` 给出原因，可能合并多项问题。
以下为典型返回；其中具体航点、节点名或底层错误会随实际情况变化。

| 情况 | 典型 `errormsg` 或输出 | 处理 |
| --- | --- | --- |
| LLM 服务不可用 | `fail: node not working: /llm_service_node (service /baidu_llm_service unavailable in this ROS domain)` | 检查环境和 LLM 服务进程 |
| 执行器不可用 | `fail: node not working: /gpsr_navigation_executor (action /gpsr/execute_actions unavailable in this ROS domain)` | 检查执行器和 ROS 域 |
| 节点没启动或不可发现 | `fail: navigation not ready: node not working: /controller_server (not discovered)` | 按返回名称检查节点；先排除 DDS/域不一致 |
| 生命周期未激活或服务不响应 | `node not working: /planner_server (lifecycle inactive)`、`(get_state unavailable)`、`(no fresh lifecycle response)` | 检查原启动日志和生命周期管理器 |
| 动态数据停止 | `node not working: /lidar_localization (no fresh data on /localization/pose_with_covariance)` | 检查底盘/雷达数据、定位及时间戳 |
| NDT 匹配不健康 | `node not working: /lidar_localization (localization unhealthy: ...)` | 恢复可靠定位后再执行 |
| TF 缺失或过期 | `TF not working: map -> odom (unavailable)`、`(stale/invalid timestamp)` | 检查对应 TF 发布者和时钟 |
| 速度链缺少端点 | `node not working: /tracer (not subscribed to /cmd_vel)` | 检查实际节点和话题映射 |
| 动作不支持 | `fail: not supported action: grasp (index 1)` | 整链拒绝，不发送任何目标 |
| 地点不存在 | `fail: unknown waypoint: bedroom` | 添加名称/别名并标定 |
| 航点未标定 | `fail: waypoint not calibrated: kitchen` | 为该航点写入真实 `x/y/yaw` |
| 文件、别名或坐标非法 | `fail: invalid waypoint configuration: ...`、`fail: invalid waypoint pose: ...` | 修正 YAML、别名冲突或非有限数值 |
| 动作格式或参数非法 | `fail: invalid action chain`、`fail: invalid parameters: lead_to (index 0)` | 使用规定的列表字符串和地点参数 |
| 航点坐标系不一致 | `fail: waypoint frame does not match navigation map frame` | 使用当前导航地图坐标系重新标定 |
| Nav2 不可用或拒绝目标 | `node not working: /bt_navigator (action /navigate_to_pose unavailable)`、`fail: Nav2 rejected the navigation goal` | 检查 Nav2 状态及日志 |
| Nav2 执行失败 | `fail: navigation failed: kitchen (Nav2 status 6)` | 状态 6 为 aborted，检查路径、代价地图和控制器 |
| 单航点执行超时 | `fail: navigation timeout: kitchen` | 当前目标请求取消，后续动作终止 |
| 整链等待超时 | `fail: navigation execution timeout; canceled` 或 `...; cancellation unconfirmed` | 确认当前目标终止状态，检查客户端与单目标超时设置 |
| 执行器忙或取消未确认 | `fail: navigation executor busy, stopping, or previous goal cancellation unconfirmed` | 等待先前目标真正结束，再提交新链 |
| 收到取消 | `fail: canceled` | 当前目标请求取消，剩余动作停止 |
| 云端解析失败 | `state: false`、`errormsg: failure` | 查看 LLM 服务日志中的鉴权、网络、HTTP 或输出格式错误 |
| 采集航点时定位不可用 | `fail: localization/base not working: no fresh TF map -> base_link` | 恢复底盘与定位 TF 后重新采集 |

`check_navigation` 返回 JSON：成功为 `state=true, errormsg="success", problems=[]`；
失败为 `state=false, errormsg="fail: navigation not ready"`，详细原因在 `problems` 数组。
它不自动启动缺失节点，也不因节点进程存在就认定数据有效。
若导航依赖与航点同时有错，执行器会同时附带相关错误。

客户端退出码：`0` 为本次调用成功，`1` 为规划/执行返回失败或被拒绝，
`2` 为服务不可用、等待超时、异常或日志写入失败，`130` 为中断。
`check_navigation` 和 `waypoint` 工具失败时使用退出码 `2`。

### 逐次命令日志与取消注意事项

`analyze_command` 和 `execute_command` 每次运行都创建独立 JSON 文档，默认目录
`~/gpsr_ros2_test/log/commands/`，文件名形如
`command_YYYYMMDD_HHMMSS_microseconds_random.json`。记录字段为：

- `command`：原始输入字符串；`mode`：`analyze` 或 `execute`。
- `status`：本次调用最终的 `success` 或 `fail`；执行模式以整条链结果为准。
- `stdout`：客户端实时输出的完整副本，执行模式包含动作链、导航进度和最终结果。
- `started_at`、`finished_at`：北京时间；`exit_code`、`error`：退出码及失败原因。

服务不可用、拒绝、超时、异常和中断也会尝试保存日志。可用 `--log-dir` 或
`GPSR_COMMAND_LOG_DIR` 改变目录；未指定时若设置了 `ROS_LOG_DIR`，使用其父目录
下的 `commands`。日志路径提示写到 stderr；`stdout` 不包含其他服务端进程的日志、
stderr 或 ros2run 退出后打印的启动器提示。

执行客户端按 Ctrl+C 会请求取消动作链和当前 Nav2 目标。返回取消请求不等于车辆
已经停止，只有目标进入终态才能确认取消处理结束。若输出
`Nav2 cancellation unconfirmed; new executions are blocked`，执行器会阻止后续链，
直到先前目标真正终止；应检查 Nav2、速度链和实际底盘状态。
执行器只允许一条链运行，调试时不要再从 RViz 或其他客户端同时发送 Nav2 目标。

本次封装完成了代码构建及真实云端 LLM、真实 Nav2 与模拟定位/底盘的链路验证；
这些结果不代表实车定位精度或硬件运动验收。实车使用前仍需实际地图下完成航点标定，
并确保底盘 CAN、雷达和定位导航模块已经运行。已有结果见
[真实云端与模拟底盘的链路记录](../../../gpsr_ros2_test/docs/validation/navigation_real_llm_report.json)。
