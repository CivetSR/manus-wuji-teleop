# Manus ↔ Wuji Hand 2 遥操作

Manus 数据手套驱动 [Wuji Hand 2](https://github.com/wuji-technology/wuji-description) 的完整开源工具链：**MuJoCo 仿真**（无真手风险）或 **Jetson 真机**。

```
Manus 手套 → manus_data_publisher (ROS2) → manus_wuji_bridge (TCP) → Wuji 手
                                              ↕ 触觉回传 → 手套振动
```

## 快速开始（仿真，推荐）

```bash
git clone https://github.com/YOUR_USER/manus-wuji-teleop.git
cd manus-wuji-teleop
bash setup.sh

# 终端 1 — Wuji 手 MuJoCo 仿真 + TCP :9500
./scripts/start_sim.sh

# 终端 2 — 无手套测试
python3 bridge/examples/x86_client_stub.py --host 127.0.0.1 --side left --demo
```

有 Manus 手套时，终端 2 改为：

```bash
source manus/scripts/env.sh
./scripts/start_manus_teleop.sh
```

## 真机（Jetson）

```bash
# Jetson 上
pip install wuji-sdk   # 或 bundle 内 wheel
./scripts/start_robot_server.sh

# x86 上
export ROBOT_HOST=<jetson-ip>
./scripts/start_manus_teleop.sh
```

## 仓库结构

| 目录 | 说明 |
|------|------|
| `bridge/` | TCP JSONL 协议、x86 桥接、Jetson 真机 server |
| `sim/` | MuJoCo 仿真 server（同协议 :9500） |
| `manus/` | Manus udev、ROS2 包、配对/可视化工具 |
| `scripts/` | 一键启动脚本 |
| `docs/` | 关节布局等 |
| `deps/` | `wuji-description`（setup 时 clone） |

## 文档

- **[CURSOR_DEPLOY.md](./CURSOR_DEPLOY.md)** — 给 Cursor / AI 的完整部署与排错指南
- [bridge/X86_MANUS_INTEGRATION.md](./bridge/X86_MANUS_INTEGRATION.md) — TCP 协议细节
- [docs/JOINT_LAYOUT.md](./docs/JOINT_LAYOUT.md) — 20 关节顺序

## 依赖

- Ubuntu 22.04 + ROS 2 Humble（Manus 真手套）
- Manus Core SDK → `~/ManusSDK`（真手套，不含在本仓库）
- Python 3.10+、`mujoco`、`pyyaml`
- [wuji-description](https://github.com/wuji-technology/wuji-description) Hand2 MJCF（自动 clone）

## License

MIT — 见 [LICENSE](./LICENSE)。Manus SDK 与 Wuji SDK 为各自厂商许可，需单独获取。
