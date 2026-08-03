# Wuji ↔ Manus Bridge（机器人端）

本目录实现 **Jetson 上的 Wuji Hand 2 网络端点**：接收远程关节命令，回传关节状态与指尖压力。  
Manus 手套必须在 **x86** 上跑 SDK；x86 侧见 **`x86/`** 与 **`X86_MANUS_INTEGRATION.md`**。

## 快速启动

**Jetson（机器人端）：**

```bash
sudo systemctl stop apex-tool.service 2>/dev/null || true
./start_server.sh
```

**x86（Manus 手套端，本机）：**

```bash
# 先确认能 ping 通 Jetson
export ROBOT_HOST=6.6.7.100
./x86/run_teleop.sh
```

或分两个终端：

```bash
source ~/srworkspace/manus-hand-viz/scripts/env.sh
ros2 run manus_ros2 manus_data_publisher

# 另一终端
export ROBOT_HOST=6.6.7.100
python3 wuji_manus_bridge/x86/manus_wuji_bridge.py
```

无 Manus 时仅测 TCP：

```bash
python3 examples/x86_client_stub.py --host 6.6.7.100 --side left --demo
```

## 首次关节标定

标定只采集 Manus 数据，不连接或驱动 Wuji。戴好手套并完成 Manus 官方校准后运行：

```bash
./x86/run_calibration.sh --side both
```

程序会依次采集自然张开、握拳、手指并拢和手指张开四个姿态，并生成：

```text
examples/retarget_manus_to_wuji.calibrated.yaml
```

标定保持角度一比一：一个 Manus 角度对应一个 Wuji 角度，只计算方向和零位，并使用 Wuji 官方机械范围限幅。测试时先单手单指验证：

```bash
./x86/run_teleop.sh \
  --config examples/retarget_manus_to_wuji.calibrated.yaml \
  --no-auto-enable
```

确认每个关节的对应关系、方向和限位后才能使能整手。左右手必须分别标定。

## 单关节安全测试

先启动 Jetson 上的 `wuji_manus_bridge` 服务，并确保急停可用、人员远离夹点。测试程序要求明确指定一只手、一个手指和一个关节：

```bash
./x86/run_joint_test.sh \
  --side left \
  --finger index \
  --joint J1 \
  --config examples/retarget_manus_to_wuji.calibrated.yaml
```

程序会读取 Wuji 实际位置，未选中的 19 个关节保持当前位置。首次目标位置差超过 `0.12 rad` 时会拒绝使能；输入屏幕显示的完整确认词后，所选关节最多运行 15 秒，按 Enter 或 `Ctrl+C` 可提前停止。

推荐按 `index J1`、`index J3`、`index J4`、`index J2` 的顺序测试，再测试其他手指；拇指最后测试。不要跳过单关节测试直接启用整手。

## 目录

| 路径 | 说明 |
|------|------|
| `robot/` | Wuji TCP 端点 |
| `protocol/schema.md` | 协议 |
| `docs/X86_CURSOR_HANDOFF.md` | x86 接续文档 |
| `x86_templates/` | 给 x86 复制的骨架 |
| `tests/mock_x86_client.py` | 无 Manus 联调客户端 |
