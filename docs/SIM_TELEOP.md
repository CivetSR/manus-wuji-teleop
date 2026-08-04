# MANUS 手套 → Wuji Hand 2 MuJoCo 仿真遥操作

本文档记录**已验证可用**的双终端启动流程：左手 MANUS 手套驱动 MuJoCo 中的 Wuji Hand 2（Beta1）仿真手。

## 架构（简要）

```text
MANUS 手套
  → manus_data_publisher (ROS2 /manus_glove_N, raw_nodes 25 点)
  → manus_wuji_bridge.py (MediaPipe 21 点 + IK)
  → TCP JSONL 127.0.0.1:9500
  → MuJoCo backend (hand2/hand2_beta1 MJCF)
```

- 数据源：**原始骨架 `raw_nodes`**，不是 ergonomics 角度。
- IK backend：默认 **`sdk`**（`wuji-sdk RetargetSession`），Hand2 Beta1 下推荐且稳定。
- 仿真与真机共用同一 TCP 协议与安全状态机；仿真无真手风险。

## 前置条件

1. 已完成一次性安装：

```bash
cd manus-wuji-teleop
export WUJI_RETARGETING_ROOT=/path/to/wuji-retargeting   # 或 clone 到 ../wuji-retargeting
bash setup.sh
bash manus/scripts/build_ros2.sh   # 需 ~/ManusSDK
```

2. MANUS dongle 已连接，手套已配对（白/蓝灯正常）。
3. **不要**同时运行 `manus/bridge/skeleton_bridge.out` 或其他占用 Manus SDK 的进程（SDK 单例）。

## 双终端启动（推荐）

### 终端 1 — Wuji Hand 2 MuJoCo 仿真 + TCP 服务

```bash
cd /path/to/manus-wuji-teleop
source scripts/activate_base.sh
HEADLESS=0 SIDES=left ./scripts/start_sim.sh
```

| 环境变量 | 默认 | 说明 |
|----------|------|------|
| `HEADLESS` | `0` | `0` 弹出 MuJoCo 窗口；`1` 无窗口 |
| `SIDES` | `left` | `left` 或 `right`（单手仿真） |
| `SIM_HOST` | `127.0.0.1` | TCP 绑定地址 |
| `SIM_PORT` | `9500` | 与 Manus 客户端一致 |

期望日志：

```text
Starting MuJoCo Wuji Hand 2 sim (left) on 127.0.0.1:9500
Official model: .../hand2/hand2_beta1/body/mjcf/left.xml
MuJoCo Hand 2 scene started ...
MuJoCo sim listening on 127.0.0.1:9500
```

### 终端 2 — MANUS 手套遥操作客户端

```bash
cd /path/to/manus-wuji-teleop
source scripts/activate_base.sh
./scripts/start_manus_teleop.sh
```

等价于（显式指定 IK backend）：

```bash
./scripts/start_manus_teleop.sh --retarget-backend sdk
```

脚本会自动：

1. 检查 `127.0.0.1:9500` 是否可达（终端 1 须先启动）
2. 启动 `manus_data_publisher`（若尚未运行）
3. 等待 `/manus_glove_*` 话题
4. 运行 `manus_wuji_bridge.py`：raw_nodes → IK → joint_cmd

**动左手套 → MuJoCo 窗口中的 Wuji Hand 2 应同步跟随。**

## 预期现象

- MuJoCo 窗口显示 **Hand 2 Beta1** 模型（解剖学 mesh，非旧版 finger1_joint 一代手）。
- 弯指、张手动作与手套一致，有约 5 Hz 低通 + 限速，动作平滑。
- 终端 2 周期性打印 teleop 状态（backend、IK 耗时、关节幅度等）。
- 若 Wuji 手有触觉回传配置，手套可能收到振动（`Vibration command sent to glove ...`）。

## 常见问题

### `waiting for valid MANUS raw_nodes; invalid=...`

- 新版只认 **25 点 raw skeleton**；ergonomics  alone 不够。
- 检查：`ros2 topic echo /manus_glove_0 --once`，确认 `raw_nodes` 非空且 `raw_node_count: 25`。
- 重新配对手套后重启 `manus_data_publisher`。
- 确保只有一个 Manus SDK 进程。

### `ERROR: Nothing listening on 127.0.0.1:9500`

- 先启动终端 1 的 `./scripts/start_sim.sh`。

### `--retarget-backend retargeter` 启动失败

- **Hand2 Beta1 模型下请用默认 `sdk`**。`retargeter`（Pinocchio）在该模型代际会因 MCP 关节方向问题 fail-fast，属预期行为。详见 [ARCHITECTURE.md](../ARCHITECTURE.md) 第 4 节。

### Python / ROS 版本错误

- 务必 `source scripts/activate_base.sh`（conda base Python 3.10 + ROS Humble）。

## 无手套自检

不依赖 MANUS 硬件，验证 IK + TCP + MuJoCo 全链路：

```bash
./scripts/run_ik_mujoco_smoke.sh
```

## 切换到真实 Hand 2

仿真验证通过后，将终端 1 换为真机 backend：

```bash
# 终端 1
./scripts/start_hand2_backend.sh

# 终端 2（不变）
./scripts/start_manus_teleop.sh
```

Hand 2 网线接交换机，x86 需能访问设备网络。

## 相关文档

| 文档 | 内容 |
|------|------|
| [README.md](../README.md) | 项目总览 |
| [ARCHITECTURE.md](../ARCHITECTURE.md) | 架构、安全状态机、IK backend 对比 |
| [CURSOR_DEPLOY.md](../CURSOR_DEPLOY.md) | Cursor / 新机器完整部署 |
| [docs/JOINT_LAYOUT.md](./JOINT_LAYOUT.md) | 20 关节顺序 |
| [bridge/protocol/schema.md](../bridge/protocol/schema.md) | TCP 消息格式 |
