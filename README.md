# Unitree G1 29DoF 强化学习策略部署

本项目使用 Unitree SDK2 的 DDS 接口读取 G1 状态、发送低层电机命令，并通过 ONNX Runtime 运行行走策略。当前主要维护的部署入口是 `robots/g1_29dof`，支持手柄控制的盲走策略和基于 ROS 2 深度图的感知行走策略。仓库还保留了其他机器人的控制器目录，但根目录脚本只构建 G1 29DoF。

深度相机通过 ROS 2 提供图像和 CameraInfo；机器人状态与电机控制仍走 Unitree SDK2。仓库内置 x86_64 和 ARM64 的 ONNX Runtime 1.22.0。x86_64 环境已完成编译及自动化测试；ARM64 板载电脑需要按下文在目标机器上重新编译和实测。

## 目录

```text
build.sh                         构建 G1 29DoF 控制器并运行测试
run_real.sh                      真机启动入口
run_real_depth.sh                检查深度话题后启动真机控制器
robots/g1_29dof/
  CMakeLists.txt                  G1 构建和安装规则
  main.cpp                       DDS 与状态机入口
  config/config.yaml             状态切换、策略路径、深度参数
  config/policy/                 ONNX 策略及部署参数
  src/                           行走、深度图、动作模仿实现
  tools/depth_capture.cpp        深度图采集工具
  tests/                         深度预处理和 ONNX 接口测试
include/                          通用状态机、观测与动作处理
thirdparty/                       x86_64 / ARM64 ONNX Runtime
```

## 环境依赖

- Linux、CMake 3.16 及以上版本、支持 C++17 的编译器。
- 已安装 Unitree SDK2、Cyclone DDS，以及 Boost.Program_options、yaml-cpp、Eigen3、fmt、spdlog。
- 使用深度相机时，板载电脑需安装 ROS 2 的 `rclcpp`、`sensor_msgs` 和能发布深度图的相机驱动；启动脚本还需要 `python3-yaml`。

项目使用的头文件和库按当前 CMake 配置从系统路径及 `/usr/local` 查找。首次构建前请确认 SDK2 和 DDS 已在目标机器安装。G1 板载环境为 **Ubuntu 20.04 + ROS 2 Foxy + ARM64/aarch64**；开发机可以使用自己的 ROS 2 安装，例如 Humble。

## 构建

在仓库根目录运行：

```bash
./build.sh
```

脚本默认构建 `g1_ctrl`、`depth_capture` 和两项测试，并运行 `ctest`。只安装 Foxy 或 Humble 其中一个时会自动识别；安装多个发行版时请明确指定。例如，G1 板载电脑使用：

```bash
UNITREE_DEPLOY_ROS_DISTRO=foxy ./build.sh
```

如果 Foxy 安装在自定义路径，可以设置 `UNITREE_DEPLOY_ROS_SETUP=/实际路径/setup.bash`。只构建不依赖 ROS 2 的控制器时使用：

```bash
UNITREE_DEPLOY_WITH_ROS2=OFF ./build.sh
```

此构建可用于盲走和动作模仿，`DepthWalk` 不可用。常用构建变量如下：

| 变量 | 作用 | 默认值 |
| --- | --- | --- |
| `UNITREE_DEPLOY_WITH_ROS2` | 编译 ROS 2 深度图后端 | `ON` |
| `UNITREE_DEPLOY_ROS_DISTRO` | 选择 `/opt/ros/<发行版>/setup.bash` | 自动识别 |
| `UNITREE_DEPLOY_ROS_SETUP` | 指定完整的 ROS 2 setup 路径 | 未设置 |
| `UNITREE_DEPLOY_BUILD_TESTS` | 构建并运行测试 | `ON` |
| `UNITREE_DEPLOY_JOBS` | 并行编译任务数 | `2` |
| `ONNXRUNTIME_ROOT` | 自定义 ONNX Runtime 安装目录 | 按 CPU 架构选择内置版本 |

默认可执行文件位于 `robots/g1_29dof/build/g1_ctrl`。控制器根据可执行文件位置寻找 `config/config.yaml`，因此运行时请使用默认 `build/` 目录或下文的安装包布局。`DEPLOY_BUILD_DIR` 虽可改变构建位置，任意目录名不一定能让控制器找到配置。

## 配置与策略

主配置文件是 [`robots/g1_29dof/config/config.yaml`](robots/g1_29dof/config/config.yaml)。其中 `FSM` 定义状态和手柄切换条件，`BlindWalk` / `DepthWalk` 分别指定 ONNX 模型与 `deploy.yaml`；`depth` 定义相机话题、裁剪、内参和超时阈值。相对策略路径以控制器的项目目录为基准解析：源码构建时是 `robots/g1_29dof`，安装后是运行包根目录。深度源也可设为 `replay` 做离线调试，此时需自行提供 `depth.replay.file` 指向的深度数据文件。

当前自带策略：

| 状态 | 策略目录 | ONNX 输入 | ONNX 输出 |
| --- | --- | --- | --- |
| `BlindWalk` | `config/policy/velocity` | `obs[1,480]` | `actions[1,29]` |
| `DepthWalk` | `config/policy/velocity-depth` | `policy[1,480]`、`depth[1,16,24,1]` | `actions[1,29]` |
| 动作模仿 | `config/policy/mimic` | 由对应策略决定 | 29 个关节动作 |

替换策略时，应同时核对模型输入输出、`deploy.yaml` 的观测与动作维度、29DoF 关节映射，以及训练时的深度预处理参数。启动时会检查行走策略文件与 ONNX 接口；深度策略不可用时，状态机会拒绝进入 `DepthWalk`。

## 仿真运行

先启动能提供 G1 29DoF Unitree DDS 话题的仿真器（例如配置好的 `unitree_mujoco`），再运行：

```bash
source /opt/ros/foxy/setup.bash  # ROS 2 构建时，替换为本机实际版本
./robots/g1_29dof/build/g1_ctrl
```

未传 `--network` 时使用 SDK2 的默认网络设置。仿真器没有发布深度图时，可以验证 `Passive`、`FixStand` 和 `BlindWalk`；进入 `DepthWalk` 仍要求配置的深度数据源已就绪。

## G1 真机运行

确认 DDS 网卡名、G1 为 29DoF 模式，并停止其他正在发布 `rt/lowcmd` 的控制程序。普通启动：

```bash
./run_real.sh eth0
```

深度行走前，先在板载电脑启动相机 ROS 2 驱动，确认它发布 `config.yaml` 中的图像和 CameraInfo 话题，再运行：

```bash
UNITREE_DEPLOY_ROS_DISTRO=foxy ./run_real_depth.sh eth0
```

示例中的 `eth0` 应替换为机器人 DDS 通信所使用的实际网卡名。

`run_real_depth.sh` 检查话题是否存在及其消息类型。深度订阅器接受 `16UC1`（毫米）或 `32FC1`（米）图像。当前配置裁剪 `[120, 110, 240, 160]`，输出 `24×16`；`strict_intrinsics: true` 要求实际 CameraInfo 经裁剪缩放后与训练相机内参相符。进入深度状态还要求新鲜且有效的深度帧；运行中断流、持续无效帧或推理故障会触发回到 `Passive`。

当前 `config.yaml` 的手柄状态切换：

| 当前状态 | 按键 | 目标状态 |
| --- | --- | --- |
| `Passive` | `LT + 上` | `FixStand` |
| `FixStand` | `RB + X` | `BlindWalk` |
| `FixStand` | `RB + Y` | `DepthWalk` |
| `BlindWalk` | `LB + X` | `FixStand` |
| `DepthWalk` | `LB + Y` | `FixStand` |
| `FixStand`、行走或动作模仿 | `LT + B` | `Passive` |

动作模仿的切换条件也写在 `config.yaml` 中。按 `Ctrl+C` 时，控制器请求切回 `Passive` 阻尼状态并退出。实机调试建议按 `Passive → FixStand → BlindWalk → DepthWalk` 的顺序验证，每一步先观察状态、关节动作和故障日志。

## 生成运行包

在目标架构机器完成构建后，从仓库根目录执行：

```bash
cmake --install robots/g1_29dof/build --prefix "$PWD/dist/g1_29dof"
```

安装目录包含 `bin/`、`lib/` 和 `config/`，可整体复制；`bin/g1_ctrl` 通过相对 `RUNPATH` 查找包内 ONNX Runtime。安装包中的启动入口为 `bin/run_real.sh` 和（启用 ROS 2 时）`bin/run_real_depth.sh`。系统仍需提供 SDK2、DDS 和 ROS 2 依赖。ARM64 构建与动态库检查步骤见 [板载部署指南](BOARD_DEPLOYMENT_ZH.md)。

## 测试与文档

重新运行自动化测试：

```bash
ctest --test-dir robots/g1_29dof/build --output-on-failure
```

深度相机采集工具的用法、输出文件和仿真对照方法见 [深度图采集指南](DEPTH_CAPTURE_GUIDE_ZH.md)。[深度部署设计记录](DEPTH_DEPLOYMENT_PLAN_ZH.md) 保留了前期设计背景；实际构建与运行命令以本 README 和板载部署指南为准。

## 常见问题

| 现象 | 检查项 |
| --- | --- |
| 找不到 ROS 2 setup | 设置 `UNITREE_DEPLOY_ROS_DISTRO` 或 `UNITREE_DEPLOY_ROS_SETUP`，确认对应文件存在。 |
| 构建时 ONNX Runtime 架构不匹配 | 确认在目标机器重新构建，并检查 `ONNXRUNTIME_ROOT` 与 `uname -m`。 |
| 启动时提示 `rt/lowcmd` 已被占用 | 停止另一个低层控制程序后再启动本程序。 |
| 不能进入 `DepthWalk` | 查看拒绝切换的日志，检查深度图、CameraInfo、内参、帧超时和策略文件。 |
