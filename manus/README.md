# Manus Metagloves Pro 手部骨架实时显示

基于 Manus SDK（Integrated 模式）采集 Metagloves Pro Haptic 手部骨架数据，并通过 PyQt5 界面实时 3D 显示。

## 架构

```
Metagloves Pro → USB Dongle → Manus SDK (C++) → UDP JSON → PyQt5 可视化
```

- `bridge/skeleton_bridge.cpp` — Manus SDK 数据桥，以 UDP JSON 广播骨架帧
- `viewer/hand_skeleton_viewer.py` — 实时 3D 手部骨架界面
- `viewer/mock_bridge.py` — 无手套时的演示数据源

## 前置条件

1. **Manus SDK**（需带 SDK 功能的 license）
   - 下载：https://docs.manus-meta.com/latest/Resources/
   - 解压到 `~/ManusSDK`

2. **系统依赖**（Ubuntu 22.04）

```bash
chmod +x scripts/setup.sh
./scripts/setup.sh
```

3. **USB 权限**：setup 脚本会安装 udev 规则；首次连接 dongle 后建议重启一次。

## 快速体验（无需手套 / SDK）

```bash
chmod +x scripts/run_mock.sh
./scripts/run_mock.sh
```

> 若 `python3` 无 PyQt5，脚本会自动使用 `python3.10`。也可手动指定：`PYTHON=python3.10 ./scripts/run_mock.sh`

界面操作：鼠标左键拖拽旋转视角，滚轮缩放。

## 连接真实手套

```bash
# 1. 编译 bridge
cd bridge
make MANUS_SDK=~/ManusSDK

# 2. 启动（自动开 bridge + 界面）
cd ..
chmod +x scripts/run.sh
./scripts/run.sh
```

或分开运行：

```bash
./bridge/skeleton_bridge.out --port 9876
python3 viewer/hand_skeleton_viewer.py --port 9876
```

## 常见问题

| 问题 | 处理 |
|------|------|
| `libManusSDK_Integrated.so not found` | 确认 SDK 路径，或 `export MANUS_SDK=/path/to/ManusSDK` |
| bridge 初始化失败 | 检查 dongle 是否插入、udev 规则是否生效 |
| 界面显示「等待手套数据」 | 确认 bridge 在运行；手套已开机并与 dongle 配对 |
| Permission denied (USB) | 运行 `./scripts/setup.sh` 并重新插拔 dongle |

## 数据格式（UDP JSON）

```json
{
  "frame": 42,
  "skeletons": [{
    "glove_id": 1,
    "nodes": [
      {"id": 0, "parent_id": -1, "side": "left", "chain": "hand", "x": 0, "y": 0, "z": 0}
    ]
  }]
}
```

默认端口：`9876`
