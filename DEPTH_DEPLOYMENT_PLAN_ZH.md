# Unitree G1 深度感知策略部署改造计划

## 0. 项目路径与参考项目

本次改造涉及的项目和参考项目如下。

### 当前训练项目

```text
/home/qihang/code/unitree_perception_lab
```

主要任务：

```text
Unitree-G1-29dof-Velocity-depth
```

关键训练代码：

```text
/home/qihang/code/unitree_perception_lab/source/unitree_rl_lab/unitree_rl_lab/tasks/locomotion/robots/g1/29dof/velocity_depth_env_cfg.py
/home/qihang/code/unitree_perception_lab/source/unitree_rl_lab/unitree_rl_lab/tasks/locomotion/agents/rsl_rl_depth_cfg.py
/home/qihang/code/unitree_perception_lab/source/unitree_rl_lab/unitree_rl_lab/tasks/locomotion/agents/rsl_rl_him_moe_cfg.py
/home/qihang/code/unitree_perception_lab/source/unitree_rl_lab/unitree_rl_lab/rsl_rl_ext/modules/actor_critic.py
/home/qihang/code/unitree_perception_lab/source/unitree_rl_lab/unitree_rl_lab/rsl_rl_ext/modules/networks.py
/home/qihang/code/unitree_perception_lab/source/unitree_rl_lab/unitree_rl_lab/rsl_rl_ext/exporters/him_moe_exporter.py
/home/qihang/code/unitree_perception_lab/source/unitree_rl_lab/unitree_rl_lab/utils/export_deploy_cfg.py
```

该项目负责：

- 训练深度感知策略；
- 导出 `policy.onnx`；
- 导出与模型对应的 `deploy.yaml`；
- 验证策略在 Isaac Lab 中的表现。

### 当前独立部署项目

```text
/home/qihang/code/unitree_deploy
```

主要改造目录：

```text
/home/qihang/code/unitree_deploy/robots/g1_29dof
```

该项目负责：

- Unitree SDK2 DDS 通信；
- 机器人本体状态获取；
- 遥控器速度指令和状态切换；
- 深度相机数据获取与预处理；
- ONNX Runtime 推理；
- 策略 action 到关节控制命令的转换；
- 软、硬、盲走和感知状态管理；
- 实机运行安全保护。

### E1 参考训练项目

```text
/home/qihang/code/TRAIN_E1ObstacleRace
```

重点参考：

- 深度策略 ONNX 的导出方式；
- 深度 Encoder、Old-HIM 和 Cross-Attention 的导出组织；
- 双输入 ONNX 的接口定义；
- 训练侧深度图尺寸和数值处理。

### E1 参考部署项目

```text
/home/qihang/code/CTRL_E1Base
```

重点参考：

- 深度图 ROS 订阅；
- `16UC1` 和 `32FC1` 深度图单位转换；
- 深度图裁剪、缩放和无效值处理；
- 盲走与感知策略切换；
- 多策略槽位和二次确认；
- 相机、遥控器、控制器和 ONNX 的协作方式；
- 实机启动脚本和安全检查。

注意：参考 E1 的部署思路，但不直接复制其电机控制架构。E1 使用 ROS Control 和 EtherCAT，G1 继续使用 Unitree SDK2 DDS 控制。

---

## 1. 改造目标

在现有 `unitree_deploy` 基础上增加 G1-29DoF 深度图感知策略部署能力，并实现类似 `CTRL_E1Base` 的控制状态切换。

最终支持以下状态：

```text
Passive     软/阻尼状态
FixStand    硬/固定站立状态
BlindWalk   盲走策略
DepthWalk   深度感知策略
```

期望状态流程：

```text
                 ┌──────── BlindWalk
Passive → FixStand
                 └──────── DepthWalk
```

第一版不建议直接在运动中执行：

```text
BlindWalk ↔ DepthWalk
```

而是先经过 `FixStand`，确保历史观测、上一帧动作和关节目标能够安全重置。

后续运行稳定后，再增加类似 E1 的“策略槽位选择 + 二次确认 + 运动中平滑切换”。

---

## 2. 当前 deploy 已有能力

现有部署程序已经具备：

- Unitree SDK2 DDS 通信；
- G1 `LowState` 接收；
- G1 `LowCmd` 发布；
- ONNX Runtime 推理；
- YAML 观测和动作配置；
- 观测历史堆叠；
- 多输入 ONNX 的基础支持；
- FSM 状态机；
- 手柄按键状态切换；
- `Passive`、`FixStand`、`Velocity` 和 `Mimic` 状态；
- 机器人姿态异常后返回 `Passive`。

现有状态大致对应：

| 状态 | 作用 |
| --- | --- |
| `Passive` | `kp=0`，保留阻尼 `kd` |
| `FixStand` | 使用固定 `kp/kd`，插值到站立姿态 |
| `Velocity` | 加载只依赖本体观测的盲走 ONNX |
| `Mimic` | 模仿动作策略 |

目前缺少：

- 深度相机数据源；
- 深度图预处理；
- `depths` 观测注册；
- 深度帧时效检查；
- 深度策略专用 FSM 状态；
- 感知策略安全切换；
- ONNX 输入输出尺寸的严格校验；
- 程序退出时的安全阻尼处理。

---

## 3. 深度策略接口约定

当前 `Unitree-G1-29dof-Velocity-depth` 导出的 ONNX 接口为：

### 输入一：本体历史

```text
名称：policy
形状：[1, 480]
含义：5帧 × 每帧96维本体观测
```

每帧 96 维包括：

```text
base_ang_vel          3
projected_gravity     3
velocity_commands     3
joint_pos_rel        29
joint_vel_rel        29
last_action          29
───────────────────────
总计                 96
```

历史帧顺序必须与训练一致：

```text
最旧帧 → 最新帧
```

### 输入二：深度图

```text
名称：depth
形状：[1, 16, 24, 1]
元素数量：384
```

深度图只使用当前帧，不进行 5 帧堆叠。

### 输出

```text
名称：actions
形状：[1, 29]
```

输出是策略原始 action，还需要经过：

```text
action scale
→ default joint position offset
→ 29维关节目标位置
→ Unitree LowCmd
```

ONNX 内部已经包含深度编码、状态估计、特征融合和策略网络，部署端不需要重复实现网络结构。

---

## 4. 深度数据源设计

建议增加与具体相机通信方式解耦的 `DepthSource` 接口。

建议目录：

```text
include/sensors/depth_source.h
include/sensors/depth_frame.h
robots/g1_29dof/include/...
robots/g1_29dof/src/...
```

接口至少提供：

```text
start()
stop()
ready()
latest()
timestamp()
sequence()
stale()
```

每帧数据至少包含：

```text
深度数据
宽度和高度
时间戳
帧序号
数据是否有效
```

策略线程不应该等待相机产生新帧，而是读取最近一帧：

```text
相机采集线程
      ↓
保存最新深度帧
      ↓
50Hz策略线程读取最近一帧
```

共享数据需要线程安全，但锁内只完成缓冲区交换，不在锁内执行裁剪、缩放和 ONNX 推理。

### 可选相机后端

建议保留统一接口，后续可以实现：

```text
RosDepthSource
RealSenseDepthSource
ReplayDepthSource
```

第一阶段建议至少实现：

1. `ReplayDepthSource`：从文件读取深度数据，用于离线测试；
2. 实机相机后端：根据实际相机驱动选择 ROS 或相机原生 SDK。

如果相机已经稳定发布 ROS 深度图，优先参考 E1 使用 ROS Subscriber。ROS 只负责传输深度图，机器人状态和电机控制仍然使用 Unitree DDS。

---

## 5. 深度图预处理

部署端必须严格复现训练时的深度观测定义。

基本流程：

```text
相机原始深度图
→ 单位转换为米
→ 裁剪有效区域
→ Resize到16×24
→ 无效值和距离范围处理
→ 减去1.0
→ 输出384个float
```

需要支持：

```text
16UC1：毫米，乘以0.001转换为米
32FC1：通常已经是米
```

当前训练范围：

```text
min_range = 0.3 m
max_range = 2.0 m
```

数值处理：

```text
NaN/Inf        → 0
小于0.3m       → 0
大于2.0m       → 0
有效值d        → d
最后统一减1.0
```

所以最终：

```text
无效深度 → -1.0
有效深度 → depth_in_meter - 1.0
```

输出布局需要与 ONNX 一致：

```text
[1, 16, 24, 1]
channels-last
row-major
```

注意：E1 使用的原图裁剪区域不能直接照搬到 G1。必须根据 G1 实际相机的以下参数重新确定裁剪区域：

- 原始分辨率；
- 相机内参；
- 安装位置；
- 安装角度；
- 机器人身体遮挡范围。

相机外参和裁剪区域是深度策略 Sim2Real 的关键参数。

---

## 6. deploy 观测系统修改

需要增加深度观测项：

```text
REGISTER_OBSERVATION(depths)
```

该观测返回当前预处理后的 384 个深度值。

推荐让 `ManagerBasedRLEnv` 持有外部观测数据提供者，例如：

```text
external_observations["depth"]
```

深度观测函数从外部数据提供者读取最新数据，而不是直接依赖 ROS 或 RealSense。

这样可以保持分层：

```text
RosDepthSource / RealSenseDepthSource
                  ↓
         外部观测缓冲区
                  ↓
REGISTER_OBSERVATION(depths)
                  ↓
ObservationManager
                  ↓
obs["depth"]
                  ↓
OrtRunner
```

`ObservationManager` 最终应生成：

```text
obs["policy"] = 480个float
obs["depth"]  = 384个float
```

`OrtRunner` 根据 ONNX 输入名称自动匹配：

```text
policy
depth
```

---

## 7. 训练侧 deploy.yaml 导出问题

训练仓库的 `export_deploy_cfg.py` 当前对多维观测的 scale 长度计算存在问题。

深度观测形状为：

```text
[N, 16, 24, 1]
```

当前逻辑会只生成 16 个 scale，而 C++ 部署观测展平后是 384 个值。部署端会拒绝这个长度不匹配的配置。

部署端使用固定的 `velocity-depth/params/deploy.yaml`，其中 `depths` 的 `scale: null` 表示单位缩放，`params: {}` 表示深度图由部署端相机预处理提供。若以后修复训练侧自动导出，深度项的维度应按下式计算：

```text
term_dim = 16 × 24 × 1 = 384
```

若导出显式 scale，应包含 384 个值。

这个修改位于当前训练项目：

```text
/home/qihang/code/unitree_perception_lab/source/unitree_rl_lab/unitree_rl_lab/utils/export_deploy_cfg.py
```

后续可将新 ONNX 以独立文件名放入 `velocity-depth/exported/`，再修改 `config.yaml` 中 `FSM.DepthWalk.model_path`。只要输入输出接口、观测顺序与含义、动作定义和深度预处理约定不变，`FSM.DepthWalk.deploy_path` 就继续指向固定的 `velocity-depth/params/deploy.yaml`。网络内部结构可以变化；上述约定变化时需要建立新的 YAML 并更新 `deploy_path`。

盲走同样通过 `FSM.BlindWalk.model_path` 和 `FSM.BlindWalk.deploy_path` 分别选择 `velocity/exported/` 中的模型和 `velocity/params/` 中的 YAML。相对路径以 `robots/g1_29dof` 为基准；控制程序启动时检查选中模型的接口和部署观测维度。

---

## 8. FSM 状态设计

建议增加四个主要状态：

```text
Passive
FixStand
BlindWalk
DepthWalk
```

### Passive

作用：

- 阻尼控制；
- 所有异常状态的最终回退状态；
- 程序启动后的默认状态。

### FixStand

作用：

- 从当前关节位置缓慢插值到站立姿态；
- 作为盲走和感知策略的安全入口；
- 作为两个策略之间切换的中间状态。

### BlindWalk

作用：

- 加载盲走 `deploy.yaml`；
- 加载盲走 `policy.onnx`；
- 只使用本体历史观测；
- 从 Unitree 遥控器读取速度指令。

### DepthWalk

作用：

- 加载深度策略 `deploy.yaml`；
- 加载双输入深度策略 `policy.onnx`；
- 使用本体历史和当前深度图；
- 检查相机连接和深度帧时效；
- 相机异常时自动返回 `Passive`。

建议策略目录：

```text
robots/g1_29dof/config/policy/
├── velocity/
│   ├── exported/
│   │   ├── policy.onnx
│   │   └── policy_v2.onnx
│   └── params/
│       ├── deploy.yaml
│       └── deploy_v2.yaml
└── velocity-depth/
    ├── exported/
    │   ├── policy.onnx
    │   └── depth_v2.onnx
    └── params/deploy.yaml
```

---

## 9. 推荐状态切换方式

第一版建议：

```text
Passive → FixStand
FixStand → BlindWalk
FixStand → DepthWalk
BlindWalk → FixStand
DepthWalk → FixStand
任意状态 → Passive
```

示例按键可以设计为：

```text
L2 + Up     Passive → FixStand
R1 + X      FixStand → BlindWalk
R1 + Y      FixStand → DepthWalk
L1 + X/Y    策略状态 → FixStand
L2 + B      任意状态 → Passive
```

最终按键可以根据实际遥控器使用习惯调整。

进入 `DepthWalk` 前必须满足：

- 深度相机已经启动；
- 已收到第一帧；
- 最新帧未超时；
- 深度数据为 384 个有限浮点数；
- ONNX 两个输入名称和尺寸正确；
- ONNX 输出为 29 维；
- 机器人姿态正常。

任意条件不满足时拒绝进入 `DepthWalk`。

退出策略状态时需要：

- 停止对应策略线程；
- 清空历史观测；
- 清空上一帧 action；
- 重新读取当前关节位置；
- 平滑进入 `FixStand` 或直接回到 `Passive`。

---

## 10. ONNX Runtime 安全检查

需要增强 `OrtRunner`：

- 校验 ONNX 输入数量；
- 校验输入名称；
- 校验输入元素数量；
- 校验固定输入形状；
- 校验输出名称和大小；
- 校验 action 是否包含 NaN/Inf；
- 捕获 ONNX Runtime 异常；
- 记录每次推理耗时；
- 推理超时后回到 `Passive`。

深度模型预期接口：

```text
policy：480
depth：384
actions：29
```

禁止在输入尺寸不匹配时继续创建 Tensor，因为这可能引起越界访问或错误推理。

---

## 11. 实机安全改造

### LowCmd 冲突

当前程序检测到其他程序正在使用 `LowCmd` 后，退出代码被注释。

需要改为：

```text
检测到其他LowCmd发布者
→ 输出明确错误
→ 立即退出
```

严禁两个控制程序同时控制机器人。

### 安全退出

增加 `SIGINT/SIGTERM` 处理：

```text
收到Ctrl+C
→ 停止策略线程
→ 切换Passive
→ 持续发送短时间阻尼命令
→ 停止DDS
→ 退出程序
```

### 深度帧超时

例如：

```text
latest_depth_age > 100ms
→ 禁止继续执行深度策略
→ 返回Passive
```

具体阈值需要根据实际相机帧率测试。

### 其他保护

需要增加或检查：

- LowState 超时；
- 机器人倾角保护；
- 关节位置软限位；
- 关节速度限制；
- action 限幅；
- 关节目标单步变化限制；
- ONNX 推理异常；
- 相机断连；
- 深度数据全无效；
- 控制线程周期严重超时。

---

## 12. 构建和启动脚本

当前项目不使用 catkin，不需要：

```bash
catkin clean
catkin build
```

建议增加根目录脚本：

```text
build.sh
run_real.sh
run_real_depth.sh
```

### build.sh

负责：

```text
创建build目录
CMake Release配置
编译g1_ctrl
```

### run_real.sh

负责：

```text
检查网卡
检查可执行文件
检查盲走模型
启动g1_ctrl
```

### run_real_depth.sh

负责：

```text
检查深度相机
检查深度模型和deploy.yaml
启动相机驱动
等待第一帧深度图
启动g1_ctrl
```

如果使用 ROS 深度图，`run_real_depth.sh` 还需要：

```text
source ROS环境
启动roscore或连接已有ROS master
启动相机节点
检查depth topic
启动g1_ctrl
```

机器人电机控制仍然走 DDS，不通过 ROS。

---

## 13. 推荐测试顺序

### 阶段一：模型文件检查

- 检查 ONNX 输入名称；
- 检查 ONNX 输入形状；
- 检查输出维度；
- 检查 deploy.yaml 观测维度；
- 检查 joint order；
- 检查 action scale 和 default joint position。

### 阶段二：深度预处理离线测试

保存一帧真实深度图，分别经过：

- Python 训练侧预处理；
- C++ 部署侧预处理。

比较最终 384 个值，确认误差在允许范围内。

### 阶段三：ONNX 推理一致性测试

使用相同的：

```text
policy[480]
depth[384]
```

分别运行：

- Python ONNX Runtime；
- C++ ONNX Runtime。

比较 29 维 action。

### 阶段四：ReplayDepthSource

不连接真实相机，从文件循环读取深度图，验证：

- 双输入 ONNX 能运行；
- FSM 能进入和退出 `DepthWalk`；
- 相机超时保护有效；
- 历史缓冲区重置正确。

### 阶段五：Sim2Sim

在 Unitree MuJoCo 中验证：

- `Passive → FixStand → BlindWalk`；
- `Passive → FixStand → DepthWalk`；
- 深度输入与仿真场景匹配；
- 异常退出可以返回 `Passive`。

### 阶段六：实机悬挂

- 机器人可靠悬挂；
- 速度指令保持为零；
- 检查关节方向；
- 检查默认关节角；
- 检查策略输出幅值；
- 测试盲走/感知切换；
- 测试相机拔出和深度超时。

### 阶段七：实机落地

- 保留安全绳；
- 从低速度开始；
- 限制前进、侧向和转向速度；
- 先测试平地；
- 再测试简单障碍物；
- 最后测试完整深度感知能力。

---

## 14. 板载电脑环境检查

部署前确认：

```bash
uname -m
```

如果输出：

```text
x86_64
```

可以使用当前：

```text
onnxruntime-linux-x64-1.22.0
```

如果输出：

```text
aarch64
```

则不能使用当前 x64 ONNX Runtime，需要替换为 ARM64 版本，并修改 CMake 路径。

还需要确认：

- Unitree SDK2 是否已经安装；
- DDS 是否可用；
- G1 29DoF `mode_machine` 是否匹配；
- 实际 DDS 网卡名称；
- 相机型号及驱动；
- ROS1、ROS2 或直接相机 SDK；
- 深度图 topic、编码和原始分辨率；
- 相机实际安装外参。

---

## 15. 预期修改文件

可能新增：

```text
include/sensors/depth_source.h
include/sensors/depth_frame.h

robots/g1_29dof/include/State_RLDepth.h
robots/g1_29dof/src/State_RLDepth.cpp

robots/g1_29dof/include/...DepthSource.h
robots/g1_29dof/src/...DepthSource.cpp

build.sh
run_real.sh
run_real_depth.sh
```

可能修改：

```text
include/isaaclab/envs/manager_based_rl_env.h
include/isaaclab/manager/observation_manager.h
include/isaaclab/algorithms/algorithms.h

robots/g1_29dof/main.cpp
robots/g1_29dof/CMakeLists.txt
robots/g1_29dof/config/config.yaml
robots/g1_29dof/src/State_RLBase.cpp
```

训练仓库还需要修改：

```text
/home/qihang/code/unitree_perception_lab/source/unitree_rl_lab/unitree_rl_lab/utils/export_deploy_cfg.py
```

---

## 16. 分阶段实施建议

### 第一阶段：安全和接口

- 修复 LowCmd 冲突后未退出；
- 增加安全退出；
- 增加 ONNX 输入输出检查；
- 确认深度 ONNX 和 deploy.yaml 接口。

### 第二阶段：深度数据链路

- 实现 `DepthSource`；
- 实现离线 Replay；
- 实现真实相机后端；
- 完成深度预处理一致性测试；
- 注册 `depths` 观测。

### 第三阶段：深度策略状态

- 增加 `DepthWalk`；
- 加载双输入 ONNX；
- 增加相机就绪和超时检查；
- 增加策略进入/退出重置。

### 第四阶段：状态切换

实现：

```text
软 → 硬 → 盲走
软 → 硬 → 感知
盲走/感知 → 硬
任意状态 → 软
```

### 第五阶段：策略槽位

运行稳定后，再参考 E1 增加：

- 多个感知策略槽位；
- 待选策略；
- 二次确认；
- ONNX 预加载；
- 速度范围随策略切换；
- 盲走和感知平滑切换。

---

## 17. 完成标准

满足以下条件才认为深度部署改造完成：

- 盲走策略仍能正常运行；
- 深度模型能够正确加载；
- ONNX 接收到 `policy[480]` 和 `depth[384]`；
- 深度预处理与训练侧一致；
- 输出为 29 维有限 action；
- 能通过手柄切换软、硬、盲走和感知状态；
- 状态切换时历史和 action 正确重置；
- 相机未启动时不能进入感知状态；
- 相机断连或深度超时时自动进入 `Passive`；
- LowState 超时时自动进入 `Passive`；
- ONNX 异常时自动进入 `Passive`；
- 其他 LowCmd 程序运行时本程序拒绝启动；
- `Ctrl+C` 能安全进入阻尼并退出；
- Sim2Sim、悬挂测试和低速落地测试全部通过。

---

## 18. 后续实施前需要确认的信息

开始实现真实相机后端之前，需要确认：

1. G1 板载电脑 CPU 架构是 `x86_64` 还是 `aarch64`；
2. 深度相机具体型号；
3. 相机驱动使用 ROS1、ROS2 还是原生 SDK；
4. 实机深度图 topic 名称；
5. 深度图编码是 `16UC1` 还是 `32FC1`；
6. 原始深度图分辨率和帧率；
7. 相机内参；
8. 相机在 G1 上的安装位置和安装角度；
9. 是否已有真实相机录制的深度图数据；
10. 最终希望使用的遥控器按键映射。

---

## 19. 总体实施路线

```text
先补安全保护
→ 固定并校验深度deploy.yaml
→ 校验ONNX接口
→ 实现DepthSource抽象
→ 完成离线深度预处理和推理测试
→ 接入真实深度相机
→ 增加DepthWalk状态
→ 实现软、硬、盲走、感知切换
→ 完成Sim2Sim和悬挂测试
→ 低速落地测试
→ 最后增加E1式多策略槽位和平滑切换
```
