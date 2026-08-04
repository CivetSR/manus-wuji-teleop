# Cursor rules for this repo

When deploying or debugging Manus ↔ Wuji teleop:

1. Read **CURSOR_DEPLOY.md** first — it is the source of truth for setup and run commands.
2. For MANUS glove + MuJoCo sim, follow **docs/SIM_TELEOP.md** (verified two-terminal workflow).
3. Never commit `~/ManusSDK`, `deps/wuji-description/`, or secrets.
4. Manus SDK is a **singleton** — only one of `manus_data_publisher` or `manus/bridge/skeleton_bridge.out` at a time.
5. TCP protocol port is **9500**; joint order is finger-major 20×rad — see `docs/JOINT_LAYOUT.md`.
6. Fix paths relative to repo root (`TELEOP_ROOT`), not `/home/omen/...`.

Quick sim test:

```bash
bash setup.sh
./scripts/start_sim.sh
python3 bridge/examples/x86_client_stub.py --host 127.0.0.1 --side left --demo
```
