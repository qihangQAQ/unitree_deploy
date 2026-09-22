# G1 深度图采集：保留最后一帧

`depth_capture` 只订阅 ROS2 相机，不发送 Unitree 电机命令。它自动读取 `robots/g1_29dof/config/config.yaml`，将数据存入 `robots/g1_29dof/depth`。采集期间持续原子更新最新处理帧；处理跟不上相机时可能跳过中间帧，但退出前会补写最后收到的完整帧。相机断流 500 ms（可配置）或采集程序收到 Ctrl+C 后，它保存带时间前缀的处理图和原始米单位图。

## 构建

在仓库根目录运行：

```bash
./build.sh
```

`depth_capture` 仅在 `UNITREE_DEPLOY_WITH_ROS2=ON` 时构建；`build.sh` 默认开启。

## 实机采集

终端 1 启动相机 ROS2 节点。终端 2 启动控制程序，让机器人到达测试位置后切回 `FixStand` 并站稳：

```bash
./run_real_depth.sh <DDS网卡名>
```

终端 3 在板载电脑实际的 `robots/g1_29dof/build` 目录运行采集程序，无需参数：

```bash
source /opt/ros/foxy/setup.bash
cd <unitree_deploy路径>/robots/g1_29dof/build
./depth_capture
```

可执行文件会从自身位置查找上一级目录中的 `config/config.yaml` 和 `depth/`；从其他工作目录启动也能定位。安装版的 `bin/`、`config/`、`depth/` 同样适用。默认订阅配置中的 `depth.topic` 和 `depth.camera_info_topic`。与控制程序一样，可以通过 `UNITREE_DEPTH_TOPIC` 和 `UNITREE_DEPTH_CAMERA_INFO_TOPIC` 覆盖话题。板载使用 Foxy 时，需要先用 Foxy 编译这个程序。

机器人站稳后，可以停止终端 1 的相机节点。终端 3 检测断流后自动退出。也可以在终端 3 按 Ctrl+C，立即保存它最后收到的完整帧。不要在 `DepthWalk` 状态下停止相机；这个状态会因深度断流触发故障保护。

## 输出

- `YYYYMMDD_HHMMSS_mmm_depth-real.csv`：16 行 × 24 列，行优先顺序。内容就是策略收到的数值：有效深度为米数加 `offset`，无效深度为 `offset`。当前配置中 `offset=-1`。
- `YYYYMMDD_HHMMSS_mmm_depth-real-raw.csv`：同一帧的原始分辨率深度图，单位为米，没有裁剪。

时间前缀使用板载电脑保存时的本地时间，精确到毫秒；若文件名碰撞，会增加序号，避免覆盖已有结果。两份 CSV 的 `#` 注释行记录帧序号、相机时间戳、编码、原始尺寸等信息。处理图还记录裁剪参数、有效像素数、CameraInfo 的 `K/P` 和转换后的 `K`。数值部分可以直接用 `numpy.loadtxt(path, delimiter=",")` 读取。

采集期间使用 `depth/` 中以 `.depth_capture_` 开头的隐藏临时文件，每次完整写入后原子替换。正常退出时将它重命名为带时间前缀的处理图；若程序意外崩溃，临时文件仍可用于排查。`depth/` 可保留之前的采集结果。默认情况下，采集程序会把 ROS 日志放在 `depth/ros_logs`。采集结果表示**采集程序最后收到的完整帧**。相机节点关闭前尚未发布的曝光不会出现在结果里。如果裁剪配置与原图尺寸不兼容，程序仍会尽量保存最终原图，方便排查。

## 与仿真数据比较

从 Isaac Lab 保存策略实际使用的深度观测 `depth`，形状同样为 `16×24`，并在制作基准数据时关闭深度噪声及相机姿态随机化。先在相同站姿、相近相机安装角度的平地场景比较热力图、有效像素区域和每行深度中位数；再加入左右偏置的已知障碍物，检查水平裁剪和镜像方向。平地一帧本身无法唯一验证水平裁剪位置。
