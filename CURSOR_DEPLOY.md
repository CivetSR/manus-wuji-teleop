# Cursor 部署指南 — Manus ↔ Wuji 遥操作

本文档供 **Cursor Agent** 在新机器上从零部署并运行。按顺序执行，不要跳过前提检查。

---

## 1. 任务目标

| 模式 | 终端 A | 终端 B |
|------|--------|--------|
| **仿真联调** | `scripts/start_sim.sh` | `scripts/start_manus_teleop.sh` 或 stub |
| **真机遥操作** | Jetson: `scripts/start_robot_server.sh` | x86: `scripts/start_manus_teleop.sh` |

协议：TCP JSONL，端口 **9500**，与 `bridge/protocol/schema.md` 一致。

---

## 2. 克隆与一键安装

```bash
git clone https://github.com/CivetSR/manus-wuji-teleop.git
cd manus-wuji-teleop
bash setup.sh
```

`setup.sh` 会：

1. 安装 apt 依赖（build-essential、libusb、ROS2 Humble 等）
2. 安装 Manus dongle udev 规则 → `/etc/udev/rules.d/70-manus-hid.rules`
3. `pip3 install -r requirements.txt`（mujoco、pyyaml）
4. clone `deps/wuji-description`（Wuji Hand2 MJCF）
5. 若存在 `~/ManusSDK`：编译 `manus_ros2` + `manus_ros2_msgs` 到 `~/ros2_ws`

---

## 3. Manus SDK（仅真手套需要）

Manus SDK **不能**随仓库分发（厂商许可）。用户需：

1. 从 Manus 获取 Linux SDK（Integrated 模式）
2. 解压后将 `SDKClient_Linux/ManusSDK` 复制到 `~/ManusSDK`
3. 重新运行：

```bash
export MANUS_SDK=~/ManusSDK
bash manus/scripts/build_ros2.sh
make -C manus/bridge MANUS_SDK=~/ManusSDK
```

验证：

```bash
source manus/scripts/env.sh
ros2 pkg list | grep manus_ros2
```

---

## 4. 仿真模式（无真手、可无手套）

### 4.1 启动 Wuji 仿真

```bash
./scripts/start_sim.sh
```

期望日志：

```
MuJoCo scene started (left hand, headless=False)
MuJoCo sim listening on 0.0.0.0:9500 (protocol v1)
```

会弹出 MuJoCo 窗口。**不要**在另一进程再跑 `manus_data_publisher`（仿真不占用 Manus SDK）。

### 4.2 无 Manus 验证 TCP

```bash
python3 bridge/examples/x86_client_stub.py --host 127.0.0.1 --side left --demo
```

仿真手小指应缓慢摆动。

### 4.3 有 Manus 手套

**终端 1** 保持 `start_sim.sh` 运行。

**终端 2**：

```bash
source manus/scripts/env.sh
./scripts/start_manus_teleop.sh
```

默认 `ROBOT_HOST=127.0.0.1`。动 glove → 仿真 Wuji 手跟随。

---

## 5. 真机模式（Jetson + Wuji Hand 2）

### Jetson（机器人端）

```bash
pip install wuji-sdk   # Hand2 以太网 SDK
sudo systemctl stop apex-tool   # 避免占用手
./scripts/start_robot_server.sh
```

### x86（Manus 端）

```bash
source manus/scripts/env.sh
export ROBOT_HOST=<jetson-ip>   # 例: 6.6.8.100
./scripts/start_manus_teleop.sh
```

---

## 6. 环境变量

| 变量 | 默认 | 说明 |
|------|------|------|
| `ROBOT_HOST` | `127.0.0.1`（teleop 脚本） | Wuji TCP 目标 |
| `ROBOT_PORT` | `9500` | TCP 端口 |
| `MANUS_SDK` | `~/ManusSDK` | Manus Integrated SDK 路径 |
| `ROS2_WS` | `~/ros2_ws` | colcon 工作空间 |
| `SIDES` | `left` | 仿真手：`left` / `right` |
| `HEADLESS` | `0` | `1` = 无 MuJoCo 窗口 |

---

## 7. 常见问题（Agent 排错清单）

### `Connection refused` on :9500

- 仿真：先启动 `scripts/start_sim.sh`
- 真机：Jetson 上 `start_robot_server.sh` 是否在跑

### `No module named 'mujoco'`

```bash
pip3 install -r requirements.txt
```

### `manus_ros2_msgs not found`

```bash
bash manus/scripts/build_ros2.sh
source manus/scripts/env.sh
```

### `/manus_glove_*` 无话题

- 手套是否配对（`manus/scripts/pair_gloves.sh`）
- dongle 是否识别：`lsusb | grep 3325`
- **不要**同时跑 `manus-hand-viz/run.sh` 与 `manus_data_publisher`（SDK 单例）

### 手套有数据但 Wuji 不动

- bridge 订阅 QoS 必须为 **RELIABLE**（已在 `bridge/x86/manus_wuji_bridge.py` 修复）
- 检查 `ros2 topic hz /manus_glove_0` 是否 ~120Hz
- 检查 bridge 日志是否有 `Teleop active: left peak=...`

### `mj_copyDataVisual ... stack is in use`

- 已修复：`sim/wuji_hand_sim/mujoco_scene.py` 在同线程执行 `mj_step` + `viewer.sync`
- 拉最新代码重启 `start_sim.sh`

### Manus SDK 与 ROS2 `set -u` 冲突

- 始终 `source manus/scripts/env.sh`（内部临时 `set +u`）

---

## 8. 关键文件索引

```
manus-wuji-teleop/
├── setup.sh                          # 首次安装
├── scripts/start_sim.sh              # MuJoCo + TCP server
├── scripts/start_manus_teleop.sh     # Manus → TCP client
├── scripts/start_robot_server.sh     # 真机 TCP server
├── bridge/x86/manus_wuji_bridge.py   # ROS2 订阅 + TCP 发送
├── bridge/x86/joint_map.py           # Ergonomics → Wuji 20 关节
├── bridge/wuji_manus_bridge/server.py # 真机 server
├── sim/wuji_hand_sim/sim_server.py   # 仿真 server
├── manus/scripts/env.sh              # ROS2 + Manus 环境
├── manus/scripts/build_ros2.sh       # 编译 manus_ros2
└── deps/wuji-description/            # Hand2 MJCF（setup 生成）
```

---

## 9. Agent 执行检查表

- [ ] `bash setup.sh` 成功
- [ ] `deps/wuji-description/hand2/.../left.xml` 存在
- [ ] `python3 -c "import mujoco"` 成功
- [ ] （真手套）`~/ManusSDK` + `build_ros2.sh` 成功
- [ ] 终端 1：`start_sim.sh` 监听 9500
- [ ] 终端 2：`start_manus_teleop.sh` 或 stub `--demo` 有响应
- [ ] （可选）`ros2 topic echo /manus_glove_0 --once` 有 ergonomics

完成以上即部署成功。
