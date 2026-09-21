# G1 深度图采集：保留最后一帧

`depth_capture` 只订阅 ROS2 相机，不发送 Unitree 电机命令。它复用部署控制程序的深度解码和预处理参数，发现新帧后原子覆盖输出目录中的 `latest_depth.csv`。处理跟不上相机时可能跳过中间帧，但退出前会补写最后收到的完整帧。相机断流 500 ms（可配置）或采集程序收到 Ctrl+C 后，它会停止订阅，并保存原始米单位图 `last_raw_depth_m.csv`。

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

终端 3 在仓库根目录运行采集程序。每次使用新的输出目录：

```bash
source /opt/ros/humble/setup.bash
./robots/g1_29dof/build/depth_capture \
  robots/g1_29dof/config/config.yaml \
  captures/flat_001
```

默认订阅 `config.yaml` 的 `depth.topic` 和 `depth.camera_info_topic`。与控制程序一样，可以通过 `UNITREE_DEPTH_TOPIC` 和 `UNITREE_DEPTH_CAMERA_INFO_TOPIC` 覆盖话题。

机器人站稳后，可以停止终端 1 的相机节点。终端 3 检测断流后自动退出。也可以在终端 3 按 Ctrl+C，立即保存它最后收到的完整帧。不要在 `DepthWalk` 状态下停止相机；这个状态会因深度断流触发故障保护。

## 输出

- `latest_depth.csv`：16 行 × 24 列，行优先顺序。内容就是策略收到的数值：有效深度为米数加 `offset`，无效深度为 `offset`。当前配置中 `offset=-1`。
- `last_raw_depth_m.csv`：最终收到的原始分辨率深度图，单位为米，没有裁剪；仅在采集结束时生成。

两份 CSV 的 `#` 注释行记录帧序号、相机时间戳、编码、原始尺寸等信息。`latest_depth.csv` 还记录裁剪参数、有效像素数、CameraInfo 的 `K/P` 和转换后的 `K`。数值部分可以直接用 `numpy.loadtxt(path, delimiter=",")` 读取。

输出目录必须不存在或为空，避免把上次采集留下的文件误当作本次结果。`latest_depth.csv` 先写同目录临时文件，再重命名覆盖；读取它时不会遇到写了一半的 CSV。默认情况下，采集程序会把 ROS 日志放在输出目录的 `ros_logs` 中。采集结果表示**采集程序最后收到的完整帧**。相机节点关闭前尚未发布的曝光不会出现在结果里。如果裁剪配置与原图尺寸不兼容，程序仍会尽量保存最终原图，方便排查。

## 与仿真数据比较

从 Isaac Lab 保存策略实际使用的深度观测 `depth`，形状同样为 `16×24`，并在制作基准数据时关闭深度噪声及相机姿态随机化。先在相同站姿、相近相机安装角度的平地场景比较热力图、有效像素区域和每行深度中位数；再加入左右偏置的已知障碍物，检查水平裁剪和镜像方向。平地一帧本身无法唯一验证水平裁剪位置。
