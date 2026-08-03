# MANUS / IK / Wuji Backend Bridge

TCP 是本机进程边界，而不是默认的跨 Jetson 链路：

```text
bridge/x86/manus_wuji_bridge.py
  -> 127.0.0.1:9500
       ├─ sim/wuji_hand_sim
       └─ bridge/wuji_manus_bridge -> wuji-sdk -> 交换机 -> Hand 2
```

两个 backend 使用相同 JSONL 协议、关节顺序、触觉消息和 shutdown 行为。

## x86 MANUS/IK client

```bash
./scripts/start_manus_teleop.sh
```

控制输入只来自 `ManusGlove.raw_nodes`。处理结构：

1. ROS callback 按 MANUS `chain_type + joint_type` 严格转换到 wrist-relative
   MediaPipe `(21,3)`（米，Y 取反）；数字 node-id 仅用于诊断。
2. callback 覆盖对应手的 latest-frame 缓存并立即返回。
3. 后台 worker 取走 pending 左右帧；两侧同时到达时并行 IK。
4. 每侧使用独立的所选 retarget backend 实例。
5. backend 统一返回 Hand2 firmware order。
6. 只发送成功、长度 20、有限的结果；坏帧不会发送零位。

Retarget 配置：

```text
config/retarget_manus_hand2_left.yaml
config/retarget_manus_hand2_right.yaml
```

配置合并了固定 `wuji-description` v2026.8.3 Beta1 URDF/MJCF/link naming 与 Host5090 的 MANUS 参数。Wuji Glove 专用的 `thumb_skip_pip`、`-90°` 旋转和 SDK per-serial offset 未复制。

## 重定向 backend

```bash
./scripts/start_manus_teleop.sh                      # sdk，默认
RETARGET_BACKEND=sdk ./scripts/start_manus_teleop.sh
```

- `sdk`（默认）：`RetargetSession.for_hand(WujiHand2, Left/Right)`，`step()` 输出直接透传，严禁重排。
- `retargeter`：`Retargeter.from_yaml`，只在此 backend 内执行一次 qpos→firmware joint-name permutation。在固定的 `hand2_beta1` 模型下 fail-fast——该代模型的 MCP 屈曲方向与官方优化器先验相反，会把关节锁死在限位。

自动 smoke 不评价动作质量。

## 本机真实 Hand2 backend

```bash
./scripts/start_hand2_backend.sh
```

默认监听 `127.0.0.1:9500`。backend 使用 `wuji_sdk`：

- `SdkManager.scan()` 过滤 `WH*`，排除 `WG*` 手套；
- 读取设备 `handedness()` 选择左右网络手；
- 可用 `WUJI_LEFT_IP/WUJI_RIGHT_IP` 显式指定地址；
- 连接后再次核对 handedness；
- 不使用 USB、`wujihandpy`、固定 SN、Jetson 网卡脚本或 `/etc/apex`。

连接设备时保持 disable；只有控制 client arm 后，控制线程才调用 `hand.enable()`。同一控制线程创建、发送并关闭 SDK joint publisher。`enable:false`、控制端断开或默认 200 ms command deadman 都会停止 publisher 发送并调用 `hand.disable()`。

可选远端运行时设置 `WUJI_BACKEND_BIND=0.0.0.0`，client 设置 `WUJI_BACKEND_HOST`。

## MuJoCo backend

```bash
HEADLESS=0 ./scripts/start_sim.sh
```

MuJoCo 从固定 commit `8271644a...` 的 `WUJI_DESCRIPTION_ROOT/hand2/hand2_beta1/body/mjcf` 读取模型。Retargeter URDF 与该 MJCF 来自同一 checkout；模型 actuator 和 joint_state 均按 joint name 映射到 TCP 顺序。

完整无手套 smoke：

```bash
./scripts/run_ik_mujoco_smoke.sh
```

## 触觉

真实 backend 将 Hand2 指尖数据转换为 `tactile` / `haptic_powers`。优先按 SDK `FingertipSensorInfo.format` v1 的 field name/type/offset/scale 完整解码；只有元数据读取不可用时才采用官方固定布局（拇指 40 点、其余各 34 点），格式存在但无效时 fail-fast，不猜测。x86 client 在 `bridge/x86/haptics.py` 中映射强度，并发布 MANUS vibration 命令。shutdown 会先停 IK worker，再清零振动、禁用 backend、关闭 TCP。

协议详见 [protocol/schema.md](./protocol/schema.md)，关节顺序详见 [../docs/JOINT_LAYOUT.md](../docs/JOINT_LAYOUT.md)。
