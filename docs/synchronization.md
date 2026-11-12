# 临时改动同步与原项目入口兼容记录

核对日期：2026-10-22。

## 项目源码

`/tmp/mid360_quickstart_fix` 是前期开发暂存区。
其中 scripts、test、launch、param、docs、rviz 共 344 个项目文件，在本次同步前
已经与 `/home/steno/lidarloc_ws/src/lidar_localization_ros2` 的对应文件逐字节一致，
没有遗漏的差异文件。实际改动及正式路径如下：

| 暂存文件 | 正式项目文件 | 处理 |
| --- | --- | --- |
| `scripts/quickstart.py` | `~/lidarloc_ws/src/lidar_localization_ros2/scripts/quickstart.py` | 本机话题默认值与类型提示已经同步，保留既有修改 |
| `test/test_quickstart_cli.py` | `~/lidarloc_ws/src/lidar_localization_ros2/test/test_quickstart_cli.py` | 配套测试已经同步 |
| `pc2_MID360_launch.py` | `~/livox_ws/src/livox_ros_driver2/launch_ROS2/pc2_MID360_launch.py` | 独立新增文件，未替换任何原启动文件 |

后续开发的 Tracer、MID360、FAST-LIO、定位与 Nav2 适配源码已直接保存在
`~/agilex_tracer_ros2_ws/src/tracer_navigation`，配置和启动文件从该包安装，
不需要 `/tmp` 下的暂存源码。

FAST-LIO 的原 `config/mid360.yaml` 和 Livox 的原 `config/MID360_config.json`
保留本机已有内容，安装副本与源码一致。适配包使用自己保存的配置副本，
不把适配包的地图保存路径或传感器外参反向覆盖到原项目默认配置。

## 原官方启动入口

核对了 Livox 仓库中全部 11 个已跟踪的 ROS 2 启动文件，以及 FAST-LIO 的
`launch/mapping.launch.py`：源码均与各自仓库 Git HEAD 一致，安装文件也与源码
一致。以下入口名称、路径和消息格式保持原样：

```bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py   # CustomMsg，xfer_format=1
ros2 launch livox_ros_driver2 rviz_MID360_launch.py  # PointCloud2，xfer_format=0
ros2 launch fast_lio mapping.launch.py config_file:=mid360.yaml
```

`msg_MID360_launch.py` SHA-256：
`11b0618d2a9a0def7f6e1a6831646d261674014344496fa1497fea67d6c0e826`

`rviz_MID360_launch.py` SHA-256：
`8331565b4c8fe1a80ec77d518b4bb2fcbe40ea8bf60a20de691d9c61417b678e`

新增 pc2 文件没有作为上述入口的替代文件、符号链接目标或重命名来源。
`tracer_navigation` 直接使用驱动可执行程序和本包参数，建图设置
`xfer_format=1`，地图定位/导航设置 `xfer_format=0`。

## 临时测试文件

`actual_generated.yaml`、`integration.yaml`、模拟地图以及 `/tmp/tracer_*.yaml`
属于生成结果或隔离测试配置，其中部分会禁用自动初始化、使用模拟数据或旧的
机器人轮廓；它们不覆盖正式硬件配置。前期临时检查程序对应的长期维护入口是
本包 `doctor`、`test/smoke_navigation.py` 和 `test/smoke_localization.py`。

成功验证的两个小型结果文件已保存到本包：

- [Nav2 模拟导航闭环](validation/navigation_smoke.json)
- [PCD 派生模拟扫描定位](validation/localization_smoke.json)

这些结果是隔离环境验证，不代表实车定位精度。网络/IP 设置保持现有值；
定位配置中的 `use_imu` 和 `use_imu_preintegration` 继续为 false。
同步文件不会配置操作系统网卡或 CAN。

## 本次复核

- 定位工作区、Livox 工作区和 tracer_navigation 均已重新构建安装。
- quickstart、配置生成与适配包的相关测试共 113 项通过。
- 原 Livox MID360、FAST-LIO mapping 以及现有定位/适配包入口的参数解析通过。
- 重建后再次逐字节核对：原 MID360 两个入口与 FAST-LIO mapping 的源码和
  安装副本仍与 Git HEAD 一致；独立 pc2 文件与暂存版本一致。

## 2026-10-28 定位启动退出修复

15:50 的单独定位日志、15:52 的组合导航日志均在启动约 3 秒后出现：

```text
launch configuration 'run_bringup_check' does not exist
```

定位节点此前已经进入 active，随后 launch 异常导致整组节点收到退出信号，
RViz 窗口因此关闭。Humble 的 `TimerAction` 保留 context locals，但不保留
已退出的 `GroupAction` 的 launch configurations；上游 quickstart 将检查开关
和话题等参数留到定时器触发时才解析，超出了本包 include 的参数作用域。

正式修复位于
[quickstart.launch.py](/home/steno/lidarloc_ws/src/lidar_localization_ros2/launch/quickstart.launch.py)：
在创建定时器前解析检查参数，将启用条件放到 `TimerAction`，保留 3 秒延迟。
安装目录中的同名文件是源码符号链接；定位工作区已重新构建安装。
本包的分组和 `/map → /localization/map_cloud` 重映射继续保留，Nav2 地图服务
不受该重映射影响。

验证：

- 本包配置及启动生命周期测试 11 项通过。新增 `test/test_launch_lifecycle.py`
  使用真实 launch 事件循环触发定时器，拦截进程创建，覆盖单独定位/组合导航、
  启用/关闭检查及地图重映射隔离；修复前能复现同一异常。
- 定位依赖包的 launch 参数、quickstart CLI/model 和 RViz 配置测试共 88 项通过，
  两个 bringup 检查入口的 `--help` 正常。
- Domain 174 的 PCD 派生扫描测试接受 55 次健康匹配，8 个 Nav2 服务 active，
  `map → odom` 与已知模拟位姿一致；激活后额外持续 4 秒，未出现延迟退出。
- Domain 176 使用默认单独定位入口实际启动定位和 RViz，持续运行 12 秒：定位
  服务保持 active，RViz 进程存活且 OpenGL 初始化正常；测试结束后主动关闭。

以上不启动底盘或雷达硬件，也不向实车 ROS 域发布测试消息。
