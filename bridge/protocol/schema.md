# Wuji Endpoint Protocol (v1)

Transport: **TCP**, host = robot Jetson (`6.6.7.100` or reachable LAN IP), default port **`9500`**.

Framing: **newline-delimited JSON** (UTF-8). Each message is one JSON object ending with `\n`.

Direction:
- **Client** = x86 Manus host
- **Server** = robot Wuji endpoint (`wuji_endpoint.py`)

---

## Connection lifecycle

1. Client TCP connect
2. Server immediately sends `hello_ack`
3. Client may send `hello` again (optional)
4. Client sends `enable` with `enabled: true`
5. Client streams `joint_cmd` / `joint_cmd_both`
6. Server streams `joint_state` + `tactile` to all connected clients
7. Client sends `enable: false` or closes socket on exit

---

## Client → Server

### `hello`

```json
{"type":"hello","client":"manus_wuji_bridge","protocol_version":1}
```

### `enable`

Required before motion if `require_enable: true` (default).

```json
{"type":"enable","enabled":true}
```

### `ping`

```json
{"type":"ping","t":1710000000.123}
```

### `get_status`

```json
{"type":"get_status"}
```

### `joint_cmd`

Drive one hand. Positions are **radians**, length **20**, finger-major:

| indices | finger |
|--------:|--------|
| 0–3 | thumb |
| 4–7 | index |
| 8–11 | middle |
| 12–15 | ring |
| 16–19 | pinky |

Within each finger, DOF order intended to align with Manus:
`MCPSpread, MCPStretch, PIPStretch, DIPStretch` (after your calibration scales/offsets).

```json
{
  "type": "joint_cmd",
  "side": "left",
  "positions_rad": [0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0],
  "t": 1710000000.123
}
```

### `joint_cmd_both`

```json
{
  "type": "joint_cmd_both",
  "left": [/* 20 floats */],
  "right": [/* 20 floats */],
  "t": 1710000000.123
}
```

---

## Server → Client

### `hello_ack`

```json
{
  "type": "hello_ack",
  "protocol_version": 1,
  "server": "wuji_endpoint",
  "enabled": false,
  "joint_count": 20,
  "finger_order": ["thumb","index","middle","ring","pinky"],
  "hands": {
    "left": {"serial":"WH2JA01260723001","connected":true},
    "right":{"serial":"WH2KA01260722001","connected":true}
  }
}
```

### `status`

```json
{"type":"status","enabled":true,"hands":{}}
```

### `pong`

```json
{"type":"pong","t":1710000000.123,"server_t":1710000000.130}
```

### `joint_state` (periodic, default 50 Hz)

```json
{
  "type": "joint_state",
  "t": 1710000000.2,
  "enabled": true,
  "hands": {
    "left": {
      "side": "left",
      "serial": "WH2JA01260723001",
      "connected": true,
      "positions_rad": [/* 20 */]
    },
    "right": { "...": "..." }
  }
}
```

### `tactile` (periodic, default 50 Hz)

Forces are **Newtons**. Enough for Manus vibration without remapping on the robot.

```json
{
  "type": "tactile",
  "t": 1710000000.2,
  "unit": "N",
  "finger_order": ["thumb","index","middle","ring","pinky"],
  "hands": {
    "left": {
      "side": "left",
      "fingers": {
        "thumb": {
          "peak_n": 0.42,
          "mean_active_n": 0.11,
          "active_points": 8,
          "point_count": 40,
          "agg": {"fx":0.0,"fy":0.0,"fz":0.35,"temp_c":31.2}
        },
        "index": {"peak_n": 0.0, "mean_active_n": 0.0, "active_points": 0, "point_count": 34, "agg": {"fx":0,"fy":0,"fz":0,"temp_c":0}},
        "middle": {"...": "..."},
        "ring": {"...": "..."},
        "pinky": {"...": "..."}
      }
    },
    "right": {"...": "..."}
  }
}
```

If server config `tactile_include_points: true`, each finger may also include `"fz":[...point forces...]`.

### `error`

```json
{"type":"error","message":"not enabled; send {\"type\":\"enable\",\"enabled\":true}"}
```

---

## Safety notes

- Endpoint refuses `joint_cmd` until `enable=true` (configurable).
- Closing the TCP connection does **not** automatically zero the hands; send `enable:false` and/or hold last safe pose before disconnect.
- Only **one** SDK client should own each hand (stop `apex-tool` / ROS wuji driver while this endpoint runs).
