# MANUS ↔ Wuji Hand 2 遥操作

MANUS 原始骨架节点可通过 `wuji-retargeting.Retargeter` 或 `wuji_sdk.RetargetSession` 驱动 Wuji Hand 2。真实手和 MuJoCo 共用同一条本机 TCP 契约：

```text
MANUS ROS2 raw_nodes
  -> MediaPipe (21,3)
  -> per-hand backend: Retargeter | RetargetSession
  -> firmware-order qpos
  -> 127.0.0.1:9500
       ├─ MuJoCo backend
       └─ x86 wuji-sdk backend -> 交换机 -> Hand 2
```

控制只使用 MANUS 原始骨架节点；无效、缺点、非有限或退化帧会被丢弃，不会转成零位命令。

## 安装

将官方仓库放在本仓库相邻目录，或设置环境变量：

```bash
export WUJI_RETARGETING_ROOT=/path/to/wuji-retargeting
bash setup.sh
```

`setup.sh` 明确激活 conda `base` Python 3.10，再 source ROS Humble；MANUS、Retargeter、MuJoCo 和 `wuji-sdk==2026.8.3` 始终使用该同一解释器。Retargeter 以 editable 方式安装，不安装其视频、相机等可选依赖。Retargeter IK 和 MuJoCo 统一使用另一个官方模型 checkout：

```text
deps/wuji-description
  v2026.8.3 / 8271644a78d69ed9a4adcf9165d882c64ad33dfa
  hand2/hand2_beta1/body/
```

若该路径已存在，`setup.sh` 会解析可能的符号链接，并校验目标是 checkout root、模型无 tracked 修改且 HEAD 恰为上述 commit；断链或错误 checkout 会直接失败，不会被脚本静默切换。

## 重定向 backend

```bash
./scripts/start_manus_teleop.sh                      # sdk，默认
RETARGET_BACKEND=sdk ./scripts/start_manus_teleop.sh # 等价
```

- `sdk`（默认）：`wuji-sdk 2026.8.3` 的 `RetargetSession.for_hand(HandModel.WujiHand2, Handedness.Left/Right)`；输出已是 firmware order，绝不再次重排。约 0.7 ms/帧。**这是固定 Hand2 Beta1 模型下唯一可用的 backend。**
- `retargeter`：Pinocchio IK + 本仓库 YAML，qpos 按 joint name 严格重排一次。**在 `hand2_beta1` 下会 fail-fast**：该模型的 `*_mcp_flex` 关节原点相对 `hand2/body` 旋转约 π，屈曲方向相反，官方优化器的"禁止负角"先验会把四个 MCP 全部锁死在 +π/2 上限。详见 [部署指南](./CURSOR_DEPLOY.md)。

自动 smoke 只验证输出有限、维度正确、链路连通和有运动，**不能判断动作质量**。

## 无手套 MuJoCo 端到端验证

```bash
./scripts/run_ik_mujoco_smoke.sh
```

脚本会启动 headless MuJoCo，生成合理的 21 点轨迹，执行官方 IK，经 localhost TCP 下发，并验证 `joint_state` 有有限且非零的变化。

也可分终端运行：

```bash
# 终端 1
HEADLESS=0 ./scripts/start_sim.sh

# 终端 2：MANUS 手套（切换 sdk 即可 A/B）
./scripts/start_manus_teleop.sh --retarget-backend retargeter
```

## 真实 Hand 2（默认同一台 x86）

Hand 2 网线接交换机，MANUS/x86 主机应能访问其网络地址。

```bash
# 终端 1：本机 TCP -> wuji-sdk -> 交换机 -> Hand 2
./scripts/start_hand2_backend.sh

# 终端 2：MANUS -> IK -> localhost TCP
./scripts/start_manus_teleop.sh
```

backend 默认扫描 `WH*` 设备并读取 handedness 选择左右手。也可显式指定：

```bash
export WUJI_LEFT_IP=<address:port>
export WUJI_RIGHT_IP=<address:port>
export WUJI_SIDES=both
./scripts/start_hand2_backend.sh
```

TCP 默认仅绑定和连接 `127.0.0.1:9500`。可选远端部署时才设置：

```bash
WUJI_BACKEND_BIND=0.0.0.0 ./scripts/start_hand2_backend.sh
WUJI_BACKEND_HOST=<remote-host> ./scripts/start_manus_teleop.sh
```

真实与 MuJoCo backend 共用同一安全状态机：TCP 连接不会使能手；首个有效 IK 结果到达后 client 才发送 arm。全 server 只允许一个控制 client；`enable:false`、控制端断开或 200 ms 无有效命令都会停止发布并 disable。命令必须恰好 20 个有限 JSON number，且 seq/时间戳严格递增、不过期。Hand2 触觉优先按 SDK format v1 的字段定义解码；仅元数据读取不可用时使用官方 40/34 点固定布局，格式存在但无效时不猜测。

## 关键实现

- `bridge/x86/manus_keypoints.py`：MANUS 25 节点到 MediaPipe 21 点，含严格校验和 Y 翻转。
- `bridge/x86/wuji_retargeting_adapter.py`：统一 per-hand 接口、两种 backend、模型 pin 与输出检查。
- `bridge/x86/joint_order.py`：Pinocchio qpos 到 Hand2 TCP/MJCF actuator 的按名 fail-fast 重排。
- `bridge/x86/manus_wuji_bridge.py`：callback 只转换/缓存，latest-only worker 并行左右 IK。
- `bridge/wuji_manus_bridge/`：共用控制状态机、本机真实 Hand2 `wuji-sdk` backend 与自描述触觉解码。
- `sim/wuji_hand_sim/`：同协议的 MuJoCo backend。

详见 [架构说明](./ARCHITECTURE.md)、[部署指南](./CURSOR_DEPLOY.md)、[关节顺序](./docs/JOINT_LAYOUT.md) 和 [bridge 文档](./bridge/README.md)。
