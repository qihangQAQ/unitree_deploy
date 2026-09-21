# G1 29DoF ARM64 板载部署

适用环境：Ubuntu 20.04、ROS 2 Foxy、ARM64/aarch64。控制器继续使用 Unitree SDK2 的 DDS 接口；深度相机驱动在板载电脑本机发布 ROS 2 话题。

## 1. 板载依赖与源码

在板载电脑确认 `uname -m` 输出 `aarch64`，并确认已安装与成功部署的 `unitree_rl_deploy` 相同的 Unitree SDK2、Cyclone DDS、C++ 系统依赖和 ROS 2 Foxy。构建深度模式需要 `rclcpp`、`sensor_msgs`；`run_real_depth.sh` 读取 YAML 时需要 `python3-yaml`。如果 Foxy 不在 `/opt/ros/foxy`，设置 `UNITREE_DEPLOY_ROS_SETUP` 为实际的 `setup.bash` 绝对路径。

把本项目源码及 `thirdparty/onnxruntime-linux-aarch64-1.22.0` 放到板载电脑，在板载电脑重新构建：

```bash
cd unitree_deploy
UNITREE_DEPLOY_ROS_DISTRO=foxy ./build.sh
```

`build.sh` 默认启用 ROS 2 和测试。板载资源受限时可设置 `UNITREE_DEPLOY_JOBS=1`。仅调试盲走且尚未准备 ROS 2 时，可用 `UNITREE_DEPLOY_WITH_ROS2=OFF ./build.sh` 构建；这类可执行文件不能进入 `DepthWalk`。

## 2. 检查 ARM64 可执行文件并生成运行目录

```bash
file robots/g1_29dof/build/g1_ctrl
ldd robots/g1_29dof/build/g1_ctrl
cmake --install robots/g1_29dof/build --prefix "$PWD/dist/g1_29dof"
readelf -d dist/g1_29dof/bin/g1_ctrl | grep RUNPATH
ldd dist/g1_29dof/bin/g1_ctrl
```

`file` 应显示 `ARM aarch64`，`ldd` 不应出现 `not found`。安装后的 `RUNPATH` 应为 `$ORIGIN/../lib`。安装目录包含 `bin/`、`lib/` 和 `config/`；必须一起复制。系统 SDK2、DDS 和 ROS 2 库仍由板载环境提供。

## 3. 相机与策略检查

先启动板载深度相机 ROS 2 驱动，检查 `config/config.yaml` 的 `depth.topic` 与 `depth.camera_info_topic`、图像类型（`16UC1` 或 `32FC1`）、尺寸、帧率和 CameraInfo。当前裁剪参数要求原图至少覆盖 `[120, 110, 240, 160]`；`strict_intrinsics: true` 要求裁剪缩放后的内参与 `target_intrinsics` 匹配。安装目录中的配置文件是运行时实际读取的配置。

可以先用不发送电机命令的采集工具验证深度链路：

```bash
source /opt/ros/foxy/setup.bash
dist/g1_29dof/bin/depth_capture dist/g1_29dof/config/config.yaml captures/board_001
```

采集和仿真深度观测的对照方法见 [DEPTH_CAPTURE_GUIDE_ZH.md](DEPTH_CAPTURE_GUIDE_ZH.md)。调好相机参数后，重新安装配置或直接编辑安装目录的 `config/config.yaml`。

## 4. 真机启动与验收

确认 DDS 网卡名，且没有其他进程发布 `rt/lowcmd`。先验证 Passive 和 FixStand，再依次验证 BlindWalk、DepthWalk；深度断流时应回到 Passive。运行：

```bash
UNITREE_DEPLOY_ROS_DISTRO=foxy dist/g1_29dof/bin/run_real_depth.sh <DDS网卡名>
```

脚本会检查配置中的两个深度话题及消息类型。控制器进入 DepthWalk 前还会检查实际帧、有效深度比例、CameraInfo、内参及超时。板载运行时观察策略单步耗时；当前策略周期为 20 ms，配置中的 `max_policy_step_ms` 为故障阈值，应依据实测数据调整。

如已有 Foxy 环境变量，也可直接运行脚本；当设备同时安装多个 ROS 2 发行版时，用 `UNITREE_DEPLOY_ROS_DISTRO` 或 `UNITREE_DEPLOY_ROS_SETUP` 明确选择。
