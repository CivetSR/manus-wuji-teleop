# X86 端：Manus 手套 ↔ 机器人 Wuji Hand 桥接开发指南

> 给 **另一台 x86 机器上的 Cursor** 使用。  
> 机器人端（Jetson / `6.6.7.100`）已实现 TCP JSONL 服务端，本文件说明你如何用 **已调试好的 Manus Core 3.1.1 SDK** 接上它。

---

## 1. 系统分工（不要搞反）

```
┌──────────────────────────────┐         TCP :9500 JSONL        ┌──────────────────────────────┐
│  x86 主机（你这边）           │ ─────────────────────────────► │  Jetson vision (机器人上位机) │
│  Manus Core + SDK            │   joint_cmd / enable            │  wuji_manus_bridge            │
│  - Ergonomics 关节角         │ ◄───────────────────────────── │  - Wuji Hand 2 ×2             │
│  - CoreSdk_VibrateFingers*   │   tactile / haptic_powers       │  - LowPass 平滑 + 触觉采集    │
└──────────────────────────────┘         joint_state             └──────────────────────────────┘
```

- **Manus SDK 只在 x86 跑**（用户明确要求）。
- Jetson **不**装 Manus；只提供动作执行 + 触觉回传。
- 机器人端默认地址：**`6.6.7.100:9500`**（以现场网为准；需与 Jetson 三层互通）。

机器人端启动（让现场同事执行）：

```bash
sudo systemctl stop apex-tool   # 避免抢 Wuji 连接
/home/nvidia/srworkspace/wuji_manus_bridge/start_server.sh
```

联通性自测（可在 x86 上跑，不依赖 Manus）：

```bash
# 把仓库里 examples/x86_client_stub.py 拷到 x86
python3 x86_client_stub.py --host 6.6.7.100 --side left --demo
```

---

## 2. 线协议（JSON Lines over TCP）

每行一个 JSON，UTF-8，`\n` 结尾。协议版本：`protocol_version = 1`。

### 2.1 握手（必须先做）

**Client → Server**

```json
{"type":"hello","protocol_version":1,"client":"manus_x86","features":["joint_cmd","tactile","haptic"]}
```

**Server → Client**（节选）

```json
{
  "type": "hello_ack",
  "protocol_version": 1,
  "hands": {
    "left":  {"serial_number":"WH2JA...","connected":true,"cutoff_hz":5.0,"control_hz":100.0},
    "right": {"serial_number":"WH2KA...","connected":true}
  },
  "joint_layout": {
    "order": "finger_major",
    "fingers": ["thumb","index","middle","ring","pinky"],
    "joints_per_finger": 4,
    "num_joints": 20,
    "index_formula": "finger*4 + joint  (joint=0..3 = J1..J4)",
    "unit": "rad"
  },
  "haptic_hint": {
    "manus_api": "CoreSdk_VibrateFingersForGlove(gloveId, float powers[5])",
    "powers_order": ["thumb","index","middle","ring","pinky"],
    "powers_range": [0.0, 1.0],
    "source_field": "tactile.haptic_powers"
  }
}
```

### 2.2 使能

```json
{"type":"enable","side":"both","enabled":true}
```

`side`: `"left"` | `"right"` | `"both"`。

### 2.3 关节命令（建议 50–100 Hz）

```json
{
  "type": "joint_cmd",
  "side": "left",
  "seq": 123,
  "t_ms": 1710000000000,
  "position": [0.0, 0.0, /* ... 共 20 个 rad ... */],
  "enable": true
}
```

- `position` **必须长度 20**（不足服务端会补 0，但映射应对齐）。
- 可选 `velocity` / `effort`（当前服务端忽略，置 0 下发）。
- 服务端会做 **LowPass(默认 5 Hz) + 角速度限幅(默认 2 rad/s)**，再以 100 Hz 发给手。  
  → 你侧 **不要**再发突变阶跃；仍建议自己做轻度平滑。

### 2.4 服务端主动推送

**关节状态**（默认 ~50 Hz）

```json
{"type":"joint_state","side":"left","seq":10,"t_ms":...,"position":[...20...],"velocity":[...],"effort":[...]}
```

**触觉 → 触觉反馈**（默认 ~50 Hz）—— **这是你驱动 Manus 振动的输入**

```json
{
  "type": "tactile",
  "side": "left",
  "seq": 42,
  "t_ms": ...,
  "fingers": [
    {"peak_n":0.12,"mean_n":0.02,"agg_fx":0.0,"agg_fy":0.0,"agg_fz":0.08,"temp_c":28.1,"active_points":3,"haptic_01":0.06},
    "... index ...",
    "... middle ...",
    "... ring ...",
    "... pinky ..."
  ],
  "haptic_powers": [0.06, 0.0, 0.0, 0.0, 0.12]
}
```

- `fingers[i]`：`i=0..4` = thumb→pinky，力单位 **N**。
- `haptic_powers[5]`：已归一化到 **[0,1]**，顺序与 Manus 一致：`{thumb,index,middle,ring,pinky}`。  
  可直接喂给 `CoreSdk_VibrateFingersForGlove`；也可自己用 `peak_n`/`agg_fz` 重映射。

### 2.5 其它

```json
{"type":"ping"}
{"type":"pong","t_ms":...}

{"type":"get_status"}
{"type":"status","ok":true,"hands":{...}}

{"type":"error","code":"...","message":"..."}
```

---

## 3. Manus SDK 你要用的数据结构

来源：`MANUS_Core_3.1.1_SDK`（本仓库旁有 zip；x86 上 SDK 已可用）。

### 3.1 手套关节角（控制 Wuji 的输入）

优先用 **Ergonomics**：

```c
// ManusSDKTypes.h
typedef enum ErgonomicsDataType {
  ErgonomicsDataType_LeftFingerThumbMCPSpread,
  ErgonomicsDataType_LeftFingerThumbMCPStretch,
  ErgonomicsDataType_LeftFingerThumbPIPStretch,
  ErgonomicsDataType_LeftFingerThumbDIPStretch,
  // Index / Middle / Ring / Pinky 同理
  // Right* 对称
  ErgonomicsDataType_MAX_SIZE
} ErgonomicsDataType;

typedef struct ErgonomicsData {
  uint32_t id;
  bool isUserID;
  float data[ErgonomicsDataType_MAX_SIZE];  // 角度，通常为度，请实测确认
} ErgonomicsData;
```

ROS2 封装（若你走 ROS2 节点）：

- `manus_ros2_msgs/ManusGlove.msg`：含 `ManusErgonomics[] ergonomics`
- `ManusErgonomics.msg`：`string type` + `float32 value`

每根手指 4 个量：`MCPSpread, MCPStretch, PIPStretch, DIPStretch`。

### 3.2 触觉振动（Wuji → Manus）

```c
// ManusSDK.h
SDKReturnCode CoreSdk_VibrateFingersForGlove(uint32_t p_GloveId, const float* p_Powers);
// p_Powers: float[5] = {thumb, index, middle, ring, pinky}, 0..1

// 或绑定 skeleton：
SDKReturnCode CoreSdk_VibrateFingersForSkeleton(uint32_t p_SkeletonId, Side p_HandType, const float* p_Powers);
```

示例逻辑见 SDKClient：

```cpp
// SDKClient.cpp HandleHapticCommands()
float t_HapticsPowers[5]; // thumb..pinky
CoreSdk_VibrateFingersForGlove(gloveId, t_HapticsPowers);
```

ROS2：`ManusVibrationCommand.msg` → `float32[5] intensities  # Thumb, Index, Middle, Ring, Pinky`

检查手套是否支持触觉：

```c
CoreSdk_DoesSkeletonGloveSupportHaptics(skeletonId, side, &isHaptics);
// Landscape 里 GloveLandscapeData.isHaptics
```

型号线索：`DeviceFamilyType_MetagloveProHaptics` / `MetagloveProPrecisionHaptics`。

---

## 4. 建议的关节映射（你必须标定）

Wuji Hand 2：**finger-major，20 关节，单位 rad**

| 下标 | 手指 | 关节 |
|------|------|------|
| 0–3 | thumb | J1–J4 |
| 4–7 | index | J1–J4 |
| 8–11 | middle | J1–J4 |
| 12–15 | ring | J1–J4 |
| 16–19 | pinky | J1–J4 |

**初始建议映射（需在实物上标定符号/偏置/缩放）：**

| Manus（每指） | → Wuji |
|---------------|--------|
| MCPSpread | J2（外展，thumb/index 尤其重要） |
| MCPStretch | J1 |
| PIPStretch | J3 |
| DIPStretch | J4 |

伪代码：

```python
def manus_ergo_to_wuji20(ergo_deg: dict, side: str) -> list[float]:
    """ergo_deg keys like 'ThumbMCPStretch' in degrees → 20 rad."""
    out = [0.0] * 20
    fingers = ["Thumb", "Index", "Middle", "Ring", "Pinky"]
    for fi, name in enumerate(fingers):
        base = fi * 4
        # 确认 Manus 是度还是弧度！SDK 文档/打印实测
        mcp_stretch = math.radians(ergo_deg[f"{name}MCPStretch"])
        mcp_spread  = math.radians(ergo_deg[f"{name}MCPSpread"])
        pip         = math.radians(ergo_deg[f"{name}PIPStretch"])
        dip         = math.radians(ergo_deg[f"{name}DIPStretch"])
        # 符号/零位：对着张开的手调到全 0；再握拳看是否同向
        out[base + 0] = scale_j1 * mcp_stretch + bias_j1
        out[base + 1] = scale_j2 * mcp_spread  + bias_j2
        out[base + 2] = scale_j3 * pip         + bias_j3
        out[base + 3] = scale_j4 * dip         + bias_j4
    return out
```

标定步骤建议：

1. 手套自然张开 → 记录 ergo → 调 `bias` 使 Wuji 也张开。  
2. 只弯某一指 PIP → 只应动 Wuji 对应 J3。  
3. 限幅到 Wuji 软限位（先用小幅度 ±0.3 rad 试）。  
4. 左右手分别标定（Manus Left* / Right* 枚举不同）。

---

## 5. 建议的 x86 程序结构

```
manus_wuji_teleop/          # 你在 x86 新建
  manus_reader.cpp/.py      # 订阅 ErgonomicsStream / ManusGlove
  retarget.py               # ergo → 20 joints（可热调 scale/bias）
  bridge_client.py          # TCP JSONL 客户端（可直接改 examples/x86_client_stub.py）
  haptic_mapper.py          # tactile.haptic_powers → VibrateFingers*
  main_loop.py              # 100 Hz：读手套 → 映射 → joint_cmd；并行读 tactile → 振动
```

主循环伪代码：

```python
# 1) connect TCP, send hello, enable both
# 2) each 10 ms:
#      ergo = manus.get_latest_ergonomics()
#      for side in (left, right):
#          pos = retarget(ergo, side)
#          send joint_cmd
# 3) on tactile message:
#      CoreSdk_VibrateFingersForGlove(glove_id[side], msg["haptic_powers"])
#      # 若无触觉手套：跳过或改 UI 提示
```

速率建议：

| 通路 | 频率 |
|------|------|
| Manus ergo → joint_cmd | 50–100 Hz |
| tactile → vibrate | 30–50 Hz（服务端已限） |
| 振动更新 | 不必超过 50 Hz |

---

## 6. 触觉映射细节

服务端默认：

```
haptic_01 = min(1.0, max(peak_n, |agg_fz|) / haptic_scale_n)
# 默认 haptic_scale_n = 2.0 N → 满振
```

你可在 x86 重算，例如非线性：

```python
def map_force(peak_n: float) -> float:
    x = max(0.0, peak_n - 0.05) / 1.5   # 死区 + 量程
    return min(1.0, x ** 0.7)
```

然后：

```c
float powers[5];
for (int i = 0; i < 5; ++i) powers[i] = mapped[i];
CoreSdk_VibrateFingersForGlove(gloveId, powers);
```

无压力时务必送 `0`，避免手套持续微振。

---

## 7. 网络与部署检查清单

- [ ] x86 能 `ping 6.6.7.100`（或现场 Jetson IP）
- [ ] `nc -vz 6.6.7.100 9500` 通
- [ ] Jetson 上 bridge 已启动，且 `apex-tool` 已停
- [ ] hello → hello_ack 里 `hands.*.connected == true`
- [ ] `--demo` stub 能安静慢速动小指（噪声大则让机器人端降 `--cutoff-hz`）
- [ ] Manus 手套 `isHaptics == true` 再开振动
- [ ] 急停/断线：TCP 断后服务端停止接收新命令（仍保持最后平滑姿态）；你侧应 `enable:false` 或停发

防火墙：放行 **TCP 9500**。

---

## 8. 机器人端文件位置（只读参考）

| 路径 | 作用 |
|------|------|
| `wuji_manus_bridge/wuji_manus_bridge/server.py` | TCP 服务 |
| `wuji_manus_bridge/wuji_manus_bridge/protocol.py` | 协议常量 |
| `wuji_manus_bridge/wuji_manus_bridge/hand_worker.py` | SDK 连接 / 控制环 |
| `wuji_manus_bridge/wuji_manus_bridge/smoother.py` | LowPass + 限速 |
| `wuji_manus_bridge/wuji_manus_bridge/tactile.py` | 指尖压力解码 |
| `wuji_manus_bridge/examples/x86_client_stub.py` | 无 Manus 的联调客户端 |
| `teleop_setup/wuji_hand2_driver_node.py` | Apex ROS 路径（同样加了平滑） |

手 SN / 网段：`/etc/apex/wuji_serial.env`，`teleop_setup/setup_wuji_hand2_network.sh`。

---

## 9. 你（x86 Cursor）的最小交付物

1. TCP 客户端（基于 stub 改）。  
2. Manus Ergonomics → `position[20]` 映射（可配置 YAML）。  
3. `tactile.haptic_powers` → `CoreSdk_VibrateFingersForGlove`。  
4. 安全：断连清零振动、可选按键总使能。  
5. 不要在 Jetson 上编译/运行 Manus SDK。

---

## 10. 已知限制

- Hand 2 当前 SDK **无**官方 `realtime_controller`；平滑在 Jetson 主机侧完成。若日后 SDK 补上，服务端会自动优先用板载滤波。  
- 关节映射 **未**在工厂标定；必须你侧调。  
- 触觉是指尖阵列摘要，不是完整 34 点云（完整点云可后续加协议字段）。  
- 与 Apex `apex-tool` **互斥**（同一时刻只能一个进程连手）。
