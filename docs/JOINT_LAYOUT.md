# Wuji Hand Joint Layout (for MuJoCo / retarget)

Source: Apex `wuji_hand_description` URDF (includes `<mujoco>` compiler tag).

## Protocol order (bridge `position[20]`, radians, finger-major)

| index | finger | joint |
|------:|--------|-------|
| 0 | thumb(finger1) | joint1 |
| 1 | thumb(finger1) | joint2 |
| 2 | thumb(finger1) | joint3 |
| 3 | thumb(finger1) | joint4 |
| 4 | index(finger2) | joint1 |
| 5 | index(finger2) | joint2 |
| 6 | index(finger2) | joint3 |
| 7 | index(finger2) | joint4 |
| 8 | middle(finger3) | joint1 |
| 9 | middle(finger3) | joint2 |
| 10 | middle(finger3) | joint3 |
| 11 | middle(finger3) | joint4 |
| 12 | ring(finger4) | joint1 |
| 13 | ring(finger4) | joint2 |
| 14 | ring(finger4) | joint3 |
| 15 | ring(finger4) | joint4 |
| 16 | pinky(finger5) | joint1 |
| 17 | pinky(finger5) | joint2 |
| 18 | pinky(finger5) | joint3 |
| 19 | pinky(finger5) | joint4 |

## Left URDF joint names (order in file)

- `left_finger1_joint1`
- `left_finger1_joint2`
- `left_finger1_joint3`
- `left_finger1_joint4`
- `left_finger1_tip_fixed`
- `left_finger2_joint1`
- `left_finger2_joint2`
- `left_finger2_joint3`
- `left_finger2_joint4`
- `left_finger2_tip_fixed`
- `left_finger3_joint1`
- `left_finger3_joint2`
- `left_finger3_joint3`
- `left_finger3_joint4`
- `left_finger3_tip_fixed`
- `left_finger4_joint1`
- `left_finger4_joint2`
- `left_finger4_joint3`
- `left_finger4_joint4`
- `left_finger4_tip_fixed`
- `left_finger5_joint1`
- `left_finger5_joint2`
- `left_finger5_joint3`
- `left_finger5_joint4`
- `left_finger5_tip_fixed`

## Notes for MuJoCo

- `left.urdf` / `right.urdf` already embed `<mujoco><compiler meshdir=.../></mujoco>`.
- Convert with your usual URDF→MJCF pipeline, or load via MuJoCo's URDF importer.
- Bridge TCP protocol: see `bridge/X86_MANUS_INTEGRATION.md`.
- Sim can implement the same JSONL server (port 9500) without real hardware.
