# x86 MANUS ↔ Wuji Hand 2 集成

## 拓扑

默认所有进程在 MANUS/x86：

```text
MANUS SDK -> ROS2 ManusGlove.raw_nodes
          -> MediaPipe 21 points
          -> Retargeter | RetargetSession
          -> localhost TCP :9500
          -> wuji-sdk backend
          -> Ethernet switch
          -> Wuji Hand 2
```

MuJoCo backend 可直接替换真实 backend，不改变 MANUS/IK client。远端 TCP backend 是可选项。

## MANUS 输入

本机 MANUS 3.1.1 实测发布 25 个节点，ID 为 `0..24`。转换按
`chain_type + joint_type` 语义选点，避免依赖不同 Core 版本之间会变化的
数字 ID；当前数字布局对应：

```python
MEDIAPIPE_TO_MANUS = (
    0, 1, 2, 3, 4,
    6, 7, 8, 9,
    11, 12, 13, 14,
    16, 17, 18, 19,
    21, 22, 23, 24,
)
```

转换读取 `node.pose.position`，减去 wrist（`Hand/Invalid`）后 Y 轴取反，
单位沿用 MANUS 骨架的米。四个非拇指 `MCP` anchor 被跳过。所有 21 个
语义点必须存在且唯一，结果必须为 `(21,3)` 且 finite；旧版数字布局
`1,22..25,3..` 只要语义字段正确也可安全解析。任何检查失败时整帧丢弃。

## Per-hand backend 与实时线程

左右手分别创建所选 backend：

```python
backend.retarget(keypoints) -> firmware_order_qpos
backend.reset()
```

- `retargeter`：从左右 YAML 调用 `Retargeter.from_yaml`；模型来自固定 `wuji-description` v2026.8.3 commit 的 `hand2/hand2_beta1/body`。
- `sdk`：调用 `RetargetSession.for_hand(HandModel.WujiHand2, Handedness.Left/Right)`。

MANUS 参数来自上一代生产调参；未混入 Wuji Glove 专用 `thumb_skip_pip` 或旋转。

ROS callback 不执行 IK 或 TCP：

```text
callback -> validate/convert -> overwrite latest(side) -> Event
worker   -> consume pending -> parallel left/right backend -> validate -> TCP
```

独立 backend 保证 warm-start 和 filter 状态不会跨手污染。worker 落后时只保留每侧最新帧。

## 关节重排

Retargeter 的 Pinocchio qpos 不是 TCP 顺序。启动时以完整 joint name 构建 permutation，并同时核对 MJCF actuator：

```text
Retargeter qpos -> name permutation -> thumb/index/middle/ring/pinky × J1..J4
```

任何名称或数量不一致均失败；没有静默 identity fallback。SDK Session 的输出契约已经是 firmware order，因此直接验证并透传，绝不套用上述 permutation。详见 `docs/JOINT_LAYOUT.md`。

选择方式：

```bash
--retarget-backend {retargeter,sdk}
RETARGET_BACKEND=sdk
```

默认 `sdk`，也是固定 `hand2_beta1` 模型下唯一可用的 backend。`retargeter` 在该模型代次下 fail-fast：其 MCP 屈曲方向与官方优化器先验相反，会把关节锁死在限位。

## 本机真实 backend

```bash
./scripts/start_hand2_backend.sh
```

backend 通过 `wuji_sdk.SdkManager` 扫描网络，只保留 `WH*` 设备，并以 `handedness()` 区分左右。可显式设置：

```bash
export WUJI_LEFT_IP=<address:port>
export WUJI_RIGHT_IP=<address:port>
```

默认 TCP bind 为 `127.0.0.1`。无需 Jetson、USB Hand、固定 SN 或专用网卡脚本。

MANUS client、Retargeter、MuJoCo 和 `wuji-sdk==2026.8.3` 统一使用 conda `base` Python 3.10；启动脚本先激活该环境，再 source ROS Humble 以加载 `rclpy`。

## 触觉与安全

- backend 回传 `tactile.haptic_powers[5]`，顺序 thumb→pinky。
- 指尖解码优先使用 SDK `FingertipSensorInfo.format` v1 的字段名、类型、offset 和 scale；仅在元数据读取不可用时使用 thumb=40 点、其余=34 点固定布局，存在但不支持的格式直接失败。
- x86 侧保留触觉逻辑于 `bridge/x86/haptics.py`。
- 无 haptic_powers 时可从指尖 peak force 重算。
- 非 finite 触觉强度转为 0。
- shutdown 顺序：停 IK worker、清零振动、disable、关闭 TCP。
- TCP command 必须恰好 20 个有限 JSON number；server 不补零、不截断、不接收布尔或字符串。
- TCP 建连不使能；只有显式 arm 才调用 `hand.enable()`。
- 全 server 只有一个控制 client；其他 client 只能读取状态。
- `enable:false`、控制端断开和默认 200 ms deadman 都停止发布并调用真实 `hand.disable()`。
- seq 与 `t_ms` 对每侧严格递增；过期、倒退和未来时间异常命令均拒绝。
- SDK joint publisher 由同一控制线程创建、发送和关闭；MuJoCo 使用完全相同的控制状态机。

## 验证

```bash
./scripts/run_tests.sh
./scripts/run_ik_mujoco_smoke.sh
```

测试不依赖真实手套；真实手使能测试需另行确认急停和夹点安全。
