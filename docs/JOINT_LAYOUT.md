# Wuji Hand 2 (Beta1) Joint Layout

Source: [wuji-description/hand2/hand2_beta1](https://github.com/wuji-technology/wuji-description/tree/main/hand2/hand2_beta1/body/mjcf)

**Do not use Hand 1** (`hand/body/mjcf` or bundle `description/urdf`) — joint names and kinematics differ.

## Protocol order (`position[20]`, radians, finger-major)

Same on TCP wire, real Hand 2 SDK, and MuJoCo sim. Index = `finger * 4 + j` where `j` = 0..3 → J1..J4.

| index | finger | J | Hand 2 MJCF joint (left) | Manus ergonomics (typical) |
|------:|--------|---|---------------------------|----------------------------|
| 0 | thumb | J1 | `l_thumb_cmc_flex` | ThumbMCPStretch |
| 1 | thumb | J2 | `l_thumb_cmc_abd` | ThumbMCPSpread |
| 2 | thumb | J3 | `l_thumb_mcp` | ThumbPIPStretch |
| 3 | thumb | J4 | `l_thumb_ip` | ThumbDIPStretch |
| 4 | index | J1 | `l_index_finger_mcp_flex` | IndexMCPStretch |
| 5 | index | J2 | `l_index_finger_mcp_abd` | IndexSpread |
| 6 | index | J3 | `l_index_finger_pip` | IndexPIPStretch |
| 7 | index | J4 | `l_index_finger_dip` | IndexDIPStretch |
| 8 | middle | J1 | `l_middle_finger_mcp_flex` | MiddleMCPStretch |
| 9 | middle | J2 | `l_middle_finger_mcp_abd` | MiddleSpread |
| 10 | middle | J3 | `l_middle_finger_pip` | MiddlePIPStretch |
| 11 | middle | J4 | `l_middle_finger_dip` | MiddleDIPStretch |
| 12 | ring | J1 | `l_ring_finger_mcp_flex` | RingMCPStretch |
| 13 | ring | J2 | `l_ring_finger_mcp_abd` | RingSpread |
| 14 | ring | J3 | `l_ring_finger_pip` | RingPIPStretch |
| 15 | ring | J4 | `l_ring_finger_dip` | RingDIPStretch |
| 16 | pinky | J1 | `l_pinky_mcp_flex` | PinkyMCPStretch |
| 17 | pinky | J2 | `l_pinky_mcp_abd` | PinkySpread |
| 18 | pinky | J3 | `l_pinky_pip` | PinkyPIPStretch |
| 19 | pinky | J4 | `l_pinky_dip` | PinkyDIPStretch |

Right hand: replace `l_` prefix with `r_`.

## MuJoCo model files (Hand 2 only)

```
deps/wuji-description/hand2/hand2_beta1/body/mjcf/left.xml   # model name: wujihand2-left
deps/wuji-description/hand2/hand2_beta1/body/mjcf/right.xml  # model name: wujihand2-right
```

Preview (official):

```bash
python -m mujoco.viewer --mjcf=deps/wuji-description/hand2/hand2_beta1/body/mjcf/left.xml
```

## Hand 1 (legacy, not used here)

Hand 1 uses `left_finger1_joint1` … naming under `hand/body/`. This repo targets **Wuji Hand 2** Ethernet hands only.
