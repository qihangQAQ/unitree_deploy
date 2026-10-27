# G1 29DoF ARM64 板载部署

适用环境：Ubuntu 20.04、ROS 2 Foxy、ARM64/aarch64。控制器继续使用 Unitree SDK2 的 DDS 接口；深度相机驱动在板载电脑本机发布 ROS 2 话题。

## 1. 板载依赖与源码

在板载电脑确认 `uname -m` 输出 `aarch64`，并确认已安装与成功部署的 `unitree_rl_deploy` 相同的 Unitree SDK2、Cyclone DDS、C++ 系统依赖和 ROS 2 Foxy。构建深度模式需要 `rclcpp`、`sensor_msgs`；`run_real_depth.sh` 读取 YAML 时需要 `python3-yaml`。如果 Foxy 不在 `/opt/ros/foxy`，设置 `UNITREE_DEPLOY_ROS_SETUP` 为实际的 `setup.bash` 绝对路径。

把本项目源码及 `thirdparty/onnxruntime-linux-aarch64-1.22.0`、`thirdparty/unitree_dds_wrapper`、`thirdparty/unitree_sdk2_hg` 放到板载电脑。后两者分别包含 G1 封装头文件和 HG 消息定义，解决系统 SDK2 未提供 `unitree/dds_wrapper/...` 或 `unitree/idl/hg/LowCmd_.hpp` 时的编译错误。HG 目录包含 `LowCmd_`、`LowState_`、`MotorCmd_`、`MotorState_`、`IMUState_` 五个原始 SDK2 头文件，适用于 x86_64 和 ARM64；仍需板载提供 SDK2 通道接口及兼容的 DDS 头文件和库。

使用 rsync 时加入 `--exclude='robots/*/build/' --exclude='dist/'`（从仓库根目录同步），避免同步开发机的 CMake 缓存及编译产物。已有的板载 `build/` 不会因排除规则而删除。若此前同步过构建目录或遇到时间戳警告，先校准板载系统时间，再在板载仓库根目录备份旧构建目录：

```bash
date
# 若时间不正确，先校准系统时间；有可用时间服务器时可启用 NTP：
sudo timedatectl set-ntp true
# 确认 date 已正确后再继续；仅在原 build 目录存在时执行：
mv robots/g1_29dof/build "robots/g1_29dof/build.backup.$(date +%Y%m%d-%H%M%S)"
```

然后在板载仓库根目录重新构建：

```bash
UNITREE_DEPLOY_ROS_DISTRO=foxy ./build.sh
```

`build.sh` 默认启用 ROS 2 和测试。板载资源受限时可设置 `UNITREE_DEPLOY_JOBS=1`。仅调试盲走且尚未准备 ROS 2 时，可用 `UNITREE_DEPLOY_WITH_ROS2=OFF ./build.sh` 构建；这类可执行文件不能进入 `DepthWalk`。

若手动调用 CMake，在板载仓库根目录执行以下命令（会自动创建 `build/`），深度模式需明确启用：

```bash
source /opt/ros/foxy/setup.bash
cmake -S robots/g1_29dof -B robots/g1_29dof/build -DUNITREE_DEPLOY_WITH_ROS2=ON
cmake --build robots/g1_29dof/build --parallel 2
ctest --test-dir robots/g1_29dof/build --output-on-failure
```

仅 `source` Foxy 不会自动启用此选项。SDK2 库由 CMake 查找后以完整路径链接，不依赖 `unitree_sdk2Config.cmake`；缺少这个配置文件本身不代表 SDK 损坏或必须重装。若仍提示该配置文件缺失，确认已同步最新的 `robots/g1_29dof/CMakeLists.txt` 并重新执行上述 CMake 命令。CMake 应输出 `Unitree HG headers: .../thirdparty/unitree_sdk2_hg/include`，缺少内置头文件时会直接提示同步哪个目录。若 make 提示文件修改时间在未来或 `Clock skew detected`，按上面的步骤校准板载时间并重新生成构建目录；这与缺少头文件是两个独立问题。若校时后源码仍带有未来时间戳，还需检查开发机时间并重新同步源码。

### SDK2 库安装在 `/opt/unitree_robotics`

`cannot find -lunitree_sdk2` 表示链接器没有找到 SDK2 库。即使已经安装 `/opt/unitree_robotics/lib/libunitree_sdk2.a`，只链接库名也可能失败。当前 CMake 会搜索标准库位置及 `/opt/unitree_robotics/lib`、`/opt/unitree_robotics/lib64`，并直接链接找到的文件；不需要额外设置 `LIBRARY_PATH` 或 `LD_LIBRARY_PATH` 来定位这个静态库。SDK 通道和 DDS C 头文件从库所属安装前缀的 `include/` 读取，DDS C++ 头文件使用同一前缀的 `include/ddscxx`，`libddscxx.so` 和 `libddsc.so` 与 SDK2 库在同一目录并以完整路径链接。

同步修改后重新运行上面的构建命令。在本次板载安装布局下，CMake 应输出：

```text
-- Unitree SDK2 library: /opt/unitree_robotics/lib/libunitree_sdk2.a
-- Unitree SDK2 headers: /opt/unitree_robotics/include
-- Unitree DDS C++ headers: /opt/unitree_robotics/include/ddscxx
-- Unitree DDS C library: /opt/unitree_robotics/lib/libddsc.so
-- Unitree DDS C++ library: /opt/unitree_robotics/lib/libddscxx.so
```

如果 SDK2 安装在其他位置，或需要切换缓存中已选定的库，可在板载显式指定文件：

```bash
UNITREE_SDK2_LIBRARY=/opt/unitree_robotics/lib/libunitree_sdk2.a \
  UNITREE_DEPLOY_ROS_DISTRO=foxy ./build.sh
```

手动调用 CMake 时使用 `-DUNITREE_SDK2_LIBRARY=/opt/unitree_robotics/lib/libunitree_sdk2.a`。该路径必须指向板载所用架构的 SDK2 库；不要同步开发机的构建缓存。SDK/DDS 头文件目录及 DDS 库路径每次配置时都根据选定的 SDK2 库重新推导，支持 `<prefix>/lib`、`<prefix>/lib64` 和 `<prefix>/lib/<architecture>` 布局。缺少 SDK2 库、配套通道头文件、DDS C/C++ 头文件或 DDS 共享库时，配置阶段会指出缺失的具体文件，不会从系统或 ROS 安装目录补用另一套依赖。

### SDK2 头文件与库接口不一致

若链接报 `undefined reference to ChannelFactory::Init(int, ... const&, bool)`，而 SDK 库仅提供两参数 `Init(int, ... const&)`，说明编译时的函数声明与库里的实现不匹配。本次板载检查确认：`/usr/local/include` 中的声明带默认参数 `bool enableSharedMemory = false`，但 `/opt/unitree_robotics` 下的头文件和库均为两参数版本。源码即使只传两个参数，使用前一份头文件编译也会生成三参数函数调用。

当前 CMake 将 SDK 通道头文件与库绑定到同一安装前缀，并向控制库、`g1_ctrl` 和控制库消费者传递该头文件路径；内置 DDS 封装和 HG 消息头文件优先级更高，ROS 深度后端仍单独编译。同步修改后，在板载项目根目录重新配置并完整编译，避免复用先前头文件生成的目标文件：

```bash
source /opt/ros/foxy/setup.bash
cmake -S robots/g1_29dof -B robots/g1_29dof/build \
  -DUNITREE_DEPLOY_WITH_ROS2=ON \
  -DUNITREE_SDK2_LIBRARY=/opt/unitree_robotics/lib/libunitree_sdk2.a
cmake --build robots/g1_29dof/build --clean-first --parallel 2
```

确认配置输出的 SDK 库和头文件均来自 `/opt/unitree_robotics`，且 `build/CMakeFiles/g1_ctrl.dir/flags.make` 中包含 `-I/opt/unitree_robotics/include`（这里的 `build/` 位于 `robots/g1_29dof/` 下）。这项修复不改变源码中的两参数 `Init` 调用。

### DDS C 与 DDS C++ 安装混用

若日志中 `/usr/local/include/ddscxx/...` 的 C++ 模板报 `iox_size`、`iox_chunk` 成员不存在，或 `ddsi_serdata_ops` 初始化项过多，同时 DDS C 头文件来自 `/opt/unitree_robotics/include/dds/...`，说明两个安装中的 DDS 头文件不匹配；同一个问题会在不同消息类型的模板展开中重复报错。

当前配置按板载 `unitree_sdk2Targets.cmake` 给出的布局，将 SDK、DDS C 和 DDS C++ 一起绑定到选定的 SDK 安装。已移除写死的 `/usr/local/include/ddscxx` 和 `/usr/local/include/iceoryx/v2.0.2`；DDS 共享库也以完整路径链接，避免只更换头文件后仍通过 `-lddsc`、`-lddscxx` 选中系统其他版本。配套 DDS 自身的头文件负责提供功能配置，不额外强制修改共享内存宏，也不修改生成的 HG 消息代码。

同步后按上一节命令重新配置并完整编译。`g1_29dof_controller_lib.dir/flags.make` 和 `g1_ctrl.dir/flags.make` 应包含 `/opt/unitree_robotics/include/ddscxx`，不再包含 `/usr/local/include/ddscxx`；`g1_ctrl.dir/link.txt` 中的 DDS 库应指向 `/opt/unitree_robotics/lib`。这些文件均位于 `robots/g1_29dof/build/CMakeFiles/`。ROS 深度后端仍仅使用 ROS 编译依赖，`depth_capture` 不链接 SDK/DDS 控制器依赖。

### SDK2 与 ROS 2 Foxy 的编译兼容

若编译日志同时出现 `/usr/local/include/dds/...` 与 `/opt/ros/foxy/include/dds/...`，并伴随 `DDSRT_SCHED_DEFAULT` 重复定义、`dds_create_topic_sertype` 未声明或 `iox_chunk` 成员不存在，说明同一编译单元混用了 SDK2 和 Foxy 的 Cyclone DDS 头文件。

当前 CMake 将 `Ros2DepthSource.cpp` 单独编译为 `g1_depth_ros2` 静态库，ROS 2 的头文件路径和编译定义只用于这个后端；机器人控制代码仅继承它的链接依赖。Foxy 的 `ament_target_dependencies` 没有 `PRIVATE` 模式，因此在后端的链接接口中使用 CMake 3.16 支持的 `LINK_ONLY` 保留最终链接所需库，同时阻止 ROS 编译依赖外传，参见 [CMake 3.16 官方说明](https://cmake.org/cmake/help/v3.16/manual/cmake-generator-expressions.7.html#output-related-expressions)。深度后端的公开头文件只使用项目类型和标准库类型；`depth_capture` 也直接链接该后端。此调整保留 ROS 2 深度功能，不需要关闭 ROS 2 选项。

同步最新 `robots/g1_29dof/CMakeLists.txt` 后重新执行 CMake，应看到 `ROS2 depth backend: isolated from Unitree DDS headers`。这是编译期头文件依赖的隔离，运行时仍需使用板载兼容的 SDK2、DDS 和 ROS 2 库。

另一个独立报错是 `ChannelPublisher::Write` 提示只接受一个参数。随项目携带的 `Publisher.h` 已改为 `Write(outgoing)`：兼容旧版 SDK2 的单参数接口，新版 SDK2 的可选等待参数默认值仍为零。需要同步 `thirdparty/unitree_dds_wrapper`，无需替换板载 SDK2 头文件。

若 `main.cpp` 报 `ChannelFactory` 没有 `Release` 成员，也是 SDK2 接口版本差异。当前入口在编译时判断是否存在 `Release()`，支持时调用，不支持时由 SDK 管理工厂生命周期。正常退出和异常退出均使用此兼容处理；原有停止状态机、停止发布线程及销毁发布/订阅对象的顺序保留。同步最新 `robots/g1_29dof/main.cpp` 后重新编译即可。

## 2. 检查 ARM64 可执行文件并生成运行目录

```bash
file robots/g1_29dof/build/g1_ctrl
ldd robots/g1_29dof/build/g1_ctrl
cmake --install robots/g1_29dof/build --prefix "$PWD/dist/g1_29dof"
readelf -d dist/g1_29dof/bin/g1_ctrl | grep RUNPATH
ldd dist/g1_29dof/bin/g1_ctrl
```

`file` 应显示 `ARM aarch64`，`ldd` 不应出现 `not found`。安装后 `g1_ctrl` 的 `RUNPATH` 应包含 `$ORIGIN/../lib` 和选定的 SDK 库目录（本次为 `/opt/unitree_robotics/lib`）。安装目录包含 `bin/`、`lib/` 和 `config/`；必须一起复制。SDK/DDS 和 ROS 2 依赖仍由板载环境提供，复制运行包时目标机器需具备同一路径的配套安装。

还要确认构建目录和安装目录中的 `g1_ctrl` 经 `ldd` 检查时，`libddsc`、`libddscxx` 均解析到本次选定的 `/opt/unitree_robotics/lib`。已设置的 `LD_LIBRARY_PATH` 优先于 `RUNPATH`；因此即使链接时使用完整路径，运行时仍可能被 `/usr/local/lib` 或 ROS 工作空间中的同名库覆盖。编译期依赖隔离不能代替板载的动态库解析检查和 ROS RMW 兼容性测试。

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
