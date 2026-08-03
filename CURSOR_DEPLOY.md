# Cursor 部署指南 — MANUS ↔ Wuji Hand 2

## 固定架构

默认所有控制进程运行在 MANUS/x86 主机：

```text
MANUS/IK client -> 127.0.0.1:9500 -> backend
                                      ├─ headless/GUI MuJoCo
                                      └─ wuji-sdk -> 交换机 -> Hand 2
```

远端 backend 仅为可选部署。不要默认假设 Jetson、USB Hand、`wujihandpy`、专用网卡脚本或 `/etc/apex` 配置。

## 1. 官方依赖与模型

本项目要求官方 `wuji-retargeting >= 2026.8.3` 的本地 checkout，并以 editable 方式安装。查找顺序：

1. `WUJI_RETARGETING_ROOT`
2. 本仓库相邻的 `../wuji-retargeting`

```bash
export WUJI_RETARGETING_ROOT=/path/to/wuji-retargeting  # 相邻目录可省略
bash setup.sh
```

安装只使用其 `pyproject.toml` 核心依赖，不安装官方 `requirements.txt` 的视频/相机可选依赖。`setup.sh` 和运行脚本明确激活 conda `base` Python 3.10，再 source ROS Humble；MANUS、Retargeter、MuJoCo 和精确固定的 `wuji-sdk==2026.8.3` 使用同一解释器。

Retargeter IK URDF 与仿真 MJCF 必须来自单独固定的模型 checkout：

```text
${WUJI_DESCRIPTION_ROOT:-deps/wuji-description}
commit 8271644a78d69ed9a4adcf9165d882c64ad33dfa (v2026.8.3)
hand2/hand2_beta1/body/
```

路径和 commit 任一不符都会启动失败，禁止混用 Hand 1 或其他 Hand2 代际。
已有路径（含符号链接目标）必须无 tracked 修改且 checkout root 正确；断链或错误现有 checkout 会 fail-fast，`setup.sh` 不会自动改写。

## 2. 重定向 backend

```bash
./scripts/start_manus_teleop.sh                      # sdk，默认
RETARGET_BACKEND=sdk ./scripts/start_manus_teleop.sh
```

- `sdk`（**默认，且是固定 Hand2 Beta1 模型下唯一可用的 backend**）：`RetargetSession.for_hand(HandModel.WujiHand2, Handedness.Left/Right)`；输出已是 firmware order，禁止再重排。实测约 0.7 ms/帧。
- `retargeter`：`Retargeter.from_yaml` + Pinocchio IK，按 joint name 重排一次。**在 `hand2_beta1` 模型下会直接 fail-fast**，不会静默输出错误动作。

`hand2_beta1` 的五个 `*_mcp_flex` 关节原点相对 `hand2/body` 旋转了约 π，屈曲正方向相反。官方 `AdaptiveOptimizerAnalytical` 的生物力学先验（`soft_min: 0.0` + `w_hyper: 1.0` 禁止负角）会因此把四个 MCP 关节全部顶到 +π/2 上限并锁死：实测整个 open→close 扫描中 7/20 个关节零位移、MCP 完全不动，而同样配置换成 `hand2/body` 就能平滑跟随（相关性 +0.947）。这不是调参能解决的——`w_dir`、`mediapipe_rotation.x`、`lp_alpha`、`segment_scaling` 四项改成官方值后钉死数量一个没变。

左右手在两种模式下均为独立状态实例。

## 3. 无手套验收

完整自动 smoke：

```bash
./scripts/run_ik_mujoco_smoke.sh                      # sdk，默认
```

成功标志：

```text
IK_TCP_MUJOCO_OK backend=<...> ... finite=True ...
joint_state_change=<positive> retarget_avg_ms=<...> retarget_max_ms=<...>
```

手动启动：

```bash
HEADLESS=0 ./scripts/start_sim.sh
source scripts/activate_base.sh
"${TELEOP_PYTHON}" bridge/examples/ik_tcp_mujoco_smoke.py \
  --host 127.0.0.1 --retarget-backend sdk
```

`HEADLESS=0` 时启动 MuJoCo viewer。默认 `SIM_HOST=127.0.0.1`、`SIM_PORT=9500`。

## 4. MANUS 手套 + MuJoCo

```bash
# 终端 1
./scripts/start_sim.sh

# 终端 2
source manus/scripts/env.sh
./scripts/start_manus_teleop.sh
```

ROS callback 只执行 raw-node 校验、MANUS→MediaPipe 转换与 latest-frame 缓存。后台 worker 运行左右独立 backend；同批左右帧并行求解，只发送成功且有限的 20 关节命令。状态日志打印 backend、每侧初始化、平均和最大耗时。

## 5. MANUS 手套 + 真实 Hand 2

确保 x86 经交换机能访问 Hand 2：

```bash
# 终端 1：本机网络手 backend
./scripts/start_hand2_backend.sh

# 终端 2：MANUS/IK client
./scripts/start_manus_teleop.sh
```

backend 默认：

- TCP 只绑定 `127.0.0.1:9500`
- `wuji_sdk.SdkManager.scan()` 过滤序列号前缀 `WH`
- 临时连接并读取 `handedness()`，分别选择左右 Hand 2
- 通过 Hand 2 网络地址连接，不使用 USB/`wujihandpy`
- 建连保持 disable；首个有效 IK 后才 arm
- 单控制 client；断连、`enable:false` 或默认 200 ms command deadman 会 disable
- SDK joint publisher 的创建、发送和关闭都归属同一控制线程
- 指尖优先按 SDK format v1 的 field type/offset/scale 解码；仅元数据读取不可用时 fallback 为 thumb 40 点、其余各 34 点，格式存在但无效则 fail-fast

指定地址：

```bash
export WUJI_LEFT_IP=<address:port>
export WUJI_RIGHT_IP=<address:port>
export WUJI_SIDES=both  # left / right / both
./scripts/start_hand2_backend.sh
```

可选远端 backend：

```bash
# backend 主机
WUJI_BACKEND_BIND=0.0.0.0 ./scripts/start_hand2_backend.sh

# MANUS/x86 主机
WUJI_BACKEND_HOST=<backend-host> ./scripts/start_manus_teleop.sh
```

## 6. MANUS SDK

MANUS SDK 为单例。`manus_data_publisher` 与其他直接打开 SDK 的进程不可同时运行。

```bash
export MANUS_SDK="${MANUS_SDK:-$HOME/ManusSDK}"
bash manus/scripts/build_ros2.sh
source manus/scripts/env.sh
ros2 run manus_ros2 manus_data_publisher
```

输入消息只保留控制所需的 `ManusGlove.raw_nodes`。MANUS 3.1.1 本机实测
发布 ID `0..24`；MediaPipe-21 对应：

```text
0,1,2,3,4, 6,7,8,9, 11,12,13,14, 16,17,18,19, 21,22,23,24
```

实际转换按 `chain_type + joint_type` 语义选点，因此也兼容旧版数字布局，
但不会猜测缺失语义。转换先减去 wrist 再将 Y 轴取反。缺点、重复点、
退化骨架、shape 错误、NaN/Inf 帧全部丢弃，状态日志会保留最后一次具体原因。

## 7. 环境变量

- `WUJI_RETARGETING_ROOT`：官方 checkout；默认相邻目录。
- `WUJI_DESCRIPTION_ROOT`：固定模型 checkout；默认 `deps/wuji-description`。
- `RETARGET_BACKEND`：`sdk`（默认）或 `retargeter`（beta1 模型下 fail-fast）。
- `WUJI_BACKEND_HOST/PORT`：client 连接；默认 `127.0.0.1:9500`。
- `WUJI_BACKEND_BIND`：真实 backend 监听；默认 `127.0.0.1`。
- `WUJI_LEFT_IP/WUJI_RIGHT_IP`：可选显式 Hand 2 地址。
- `WUJI_SIDES`：真实 backend 手侧；默认 `both`。
- `WUJI_COMMAND_TIMEOUT_MS`：command deadman；默认 `200`，允许 `100..250`。
- `SIM_HOST/SIM_PORT`：MuJoCo backend；默认 `127.0.0.1:9500`。
- `SIDES`：MuJoCo 单手场景，`left` 或 `right`；默认 `left`（不静默降级 `both`）。
- `HEADLESS=1`：禁用 MuJoCo viewer；缺省或 `HEADLESS=0` 启动有头 viewer。
- `CONDA_ROOT`：conda 安装根目录；默认 `${HOME}/miniconda3`。所有脚本使用其 `base` Python 3.10。

## 8. 排错

### backend connection refused

先启动且只启动一个 backend：

```bash
./scripts/start_sim.sh
# 或
./scripts/start_hand2_backend.sh
```

### 找不到 Hand 2

- 确认 x86 与 Hand 2 经交换机路由可达。
- 运行 `python3 -c "from wuji_sdk import SdkManager; print(SdkManager.instance().scan())"`。
- 多手时检查 handedness；也可设置 `WUJI_LEFT_IP/WUJI_RIGHT_IP`。
- 确认没有其他 SDK session 占用手。

### 有手套数据但不发送

查看 bridge 状态日志的 `invalid`、`latest_dropped`、`failed` 和 IK 时间。无效 raw-node 帧不会发送零命令。

### 模型或顺序错误

启动会同时验证：

- imported editable package 与 `WUJI_RETARGETING_ROOT` 是同一 checkout
- `WUJI_DESCRIPTION_ROOT` 恰为固定 commit
- 配置 URDF 与 MJCF 都是该 checkout 的 `hand2/hand2_beta1/body`
- Pinocchio qpos 名称与 20 个 TCP 名称集合完全一致
- MJCF actuator 顺序与 Hand2 device/TCP 顺序一致

任何不一致均直接失败，不做 identity fallback。

## 9. 验收

```bash
./scripts/run_tests.sh
"${TELEOP_PYTHON}" -m compileall -q bridge sim tests
./scripts/run_ik_mujoco_smoke.sh
```

真实手测试必须在人员远离夹点、急停可用时另行执行；MuJoCo smoke 不会连接真实手。
