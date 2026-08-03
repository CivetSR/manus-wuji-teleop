# Wuji Hand 2 关节与模型顺序

## 模型代际

Retargeter IK 与 MuJoCo 均使用官方 `wuji-description` 固定资产：

```text
v2026.8.3
commit 8271644a78d69ed9a4adcf9165d882c64ad33dfa
hand2/hand2_beta1/body/urdf/{left,right}.urdf
hand2/hand2_beta1/body/mjcf/{left,right}.xml
```

默认 checkout 为 `deps/wuji-description`，也可用 `WUJI_DESCRIPTION_ROOT` 指定同一 commit。不使用 Hand 1 或其他 Hand2 代际。

## TCP、真实 Hand 2 与 MJCF actuator 顺序

协议是 20×rad、finger-major：

| index | finger | joint | 左手模型名 |
|---:|---|---|---|
| 0 | thumb | J1 | `l_thumb_cmc_flex` |
| 1 | thumb | J2 | `l_thumb_cmc_abd` |
| 2 | thumb | J3 | `l_thumb_mcp` |
| 3 | thumb | J4 | `l_thumb_ip` |
| 4 | index | J1 | `l_index_finger_mcp_flex` |
| 5 | index | J2 | `l_index_finger_mcp_abd` |
| 6 | index | J3 | `l_index_finger_pip` |
| 7 | index | J4 | `l_index_finger_dip` |
| 8 | middle | J1 | `l_middle_finger_mcp_flex` |
| 9 | middle | J2 | `l_middle_finger_mcp_abd` |
| 10 | middle | J3 | `l_middle_finger_pip` |
| 11 | middle | J4 | `l_middle_finger_dip` |
| 12 | ring | J1 | `l_ring_finger_mcp_flex` |
| 13 | ring | J2 | `l_ring_finger_mcp_abd` |
| 14 | ring | J3 | `l_ring_finger_pip` |
| 15 | ring | J4 | `l_ring_finger_dip` |
| 16 | pinky | J1 | `l_pinky_mcp_flex` |
| 17 | pinky | J2 | `l_pinky_mcp_abd` |
| 18 | pinky | J3 | `l_pinky_pip` |
| 19 | pinky | J4 | `l_pinky_dip` |

右手将 `l_` 替换为 `r_`。MJCF 的 20 个 position actuator 已实测按上述顺序声明；Hand 2 SDK 的 joint `nid=0..19` 使用同一 finger-major 契约。

## Retargeter 实际 qpos 顺序

官方 `Retargeter` 输出底层 Pinocchio qpos 顺序，而当前 Hand2 URDF 的顺序为：

```text
index[4], middle[4], pinky[4], ring[4], thumb[4]
```

因此不能直接发 TCP。当前官方模型对应的 qpos→TCP permutation 为：

```text
[16,17,18,19, 0,1,2,3, 4,5,6,7, 12,13,14,15, 8,9,10,11]
```

`bridge/x86/joint_order.py` 仍按完整 joint name 动态构建 permutation；上面的常量只用于文档和测试。名称缺失、重复、数量不一致或模型代际混用时启动直接失败。即使结果恰好是 identity，也必须先通过名称集合校验。

MuJoCo scene 同样按名称建立 TCP→actuator 和 joint_state→TCP 映射，不依赖数组下标猜测。

## SDK RetargetSession 顺序

`wuji-sdk 2026.8.3` 的 `RetargetSession.step()` 明确返回 `(20,) float32` firmware order。`sdk` backend 只做 shape/finite 校验并直接透传，不能再次应用上面的 Retargeter permutation。
