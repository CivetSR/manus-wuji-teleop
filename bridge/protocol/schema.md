# Wuji Backend TCP JSONL Protocol v1

默认 transport 为本机 `127.0.0.1:9500`。client 是 MANUS/IK bridge；server 是同一 x86 上的 MuJoCo 或 `wuji-sdk` Hand2 backend。可选远端部署不改变协议。

每个 UTF-8 JSON object 以 `\n` 结尾。

## 生命周期

1. client 连接并发送 `hello`
2. server 返回 `hello_ack`
3. client 发送 `enable:true`，取得唯一控制租约并 arm 指定手
4. client 发送每侧 `joint_cmd`
5. server 推送 `joint_state` 和 `tactile`
6. shutdown 时 client 发送 `enable:false`

## Client → server

```json
{"type":"hello","client":"manus_wuji_bridge","protocol_version":1,"features":["joint_cmd","tactile","haptic"]}
```

```json
{"type":"enable","side":"both","enabled":true}
```

```json
{
  "type":"joint_cmd",
  "side":"left",
  "seq":123,
  "t_ms":1710000000000,
  "position":[0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0],
  "enable":true
}
```

`position` 必须恰好 20 个 JSON number、全部有限、单位 rad。布尔值和数字字符串也拒绝；server 不截断、不补零、不做类型强转。

`seq` 和 `t_ms` 都是必填的非负整数，并且对每侧严格递增。默认拒绝超过 250 ms 的旧命令和超过 1000 ms 的未来时间戳。`joint_cmd.enable:true` 不会替代上一步显式 arm；`joint_cmd.enable:false` 等价于立即 disarm，且不执行其中的位置。

## 控制安全状态机

- 同一时刻全 server 只有一个 client 可持有控制租约；其他连接仍可读取状态，但 `enable:true` 返回 `control_busy`，写命令返回 `not_controller`。
- TCP 建连和 `hello` 都不会使能手。只有控制 owner 的显式 `enable:true` 才会 arm。
- 每侧从 arm 起及最后一条有效命令起使用默认 200 ms deadman（可配置范围 100–250 ms）。
- `enable:false`、owner 断连或 deadman 到期都会停止发布，并对真实 Hand 2 调用 `hand.disable()`；MuJoCo 使用同一状态机。
- deadman 后必须重新 arm。坏命令、过期命令和第二 client 的命令不会刷新 deadman。

顺序为 finger-major：

```text
0..3 thumb, 4..7 index, 8..11 middle, 12..15 ring, 16..19 pinky
```

每指 J1..J4 的模型名见 `docs/JOINT_LAYOUT.md`。

其他请求：

```json
{"type":"ping"}
{"type":"get_status"}
```

## Server → client

```json
{
  "type":"hello_ack",
  "protocol_version":1,
  "server":"wuji_manus_bridge",
  "command_timeout_ms":200,
  "max_command_age_ms":250,
  "hands":{
    "left":{"connected":true,"address":"192.168.1.10:5000","serial_number":"WH..."}
  },
  "joint_layout":{
    "order":"finger_major",
    "fingers":["thumb","index","middle","ring","pinky"],
    "joints_per_finger":4,
    "num_joints":20,
    "unit":"rad"
  }
}
```

```json
{
  "type":"joint_state",
  "side":"left",
  "seq":10,
  "t_ms":1710000000000,
  "position":[0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0],
  "velocity":[],
  "effort":[]
}
```

```json
{
  "type":"tactile",
  "side":"left",
  "seq":42,
  "t_ms":1710000000000,
  "fingers":[
    {"peak_n":0.42,"mean_n":0.11,"agg_fz":0.35,"point_count":40,"haptic_01":0.21}
  ],
  "haptic_powers":[0.21,0.0,0.0,0.0,0.0]
}
```

`haptic_powers` 顺序 thumb→pinky，范围 `[0,1]`。真实 backend 启动时优先读取 SDK `FingertipSensorInfo.format` v1，并按其中每个 field 的 name/type/offset/scale 解码；只有元数据读取不可用时才采用官方 Hand 2 固定布局：拇指 40 点，其余四指各 34 点。存在但不支持的格式会使该手启动失败；长度、finite 或 metadata digest 不匹配的帧会丢弃。MuJoCo backend 返回零触觉。

错误示例：

```json
{"type":"error","code":"bad_position","message":"position must be a JSON list of exactly 20 values"}
```
