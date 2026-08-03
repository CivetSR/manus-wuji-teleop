"""Host-side command smoother for Wuji Hand 2.

Official Wuji Hand (v1) uses onboard realtime_controller + LowPass (16 kHz
motor-side filter). Wuji Hand 2 (Ethernet) in wuji-sdk 2026.7.21 does NOT
expose realtime_controller, so we approximate the documented behaviour:

  - first-order low-pass on target positions (default cutoff_hz=5.0)
  - per-joint slew-rate limit (max_joint_speed_rad_s)
  - fixed-rate publish loop (default 100 Hz)

Refs:
  https://docs.wuji.tech/docs/en/wuji-sdk/latest/quick-start/
  https://docs.wuji.tech/docs/en/wuji-hand/v1/sdk-user-guide/tutorial/
"""

from __future__ import annotations

import math
from typing import List, Optional


class JointSmoother:
    def __init__(
        self,
        num_joints: int = 20,
        cutoff_hz: float = 5.0,
        control_hz: float = 100.0,
        max_joint_speed_rad_s: float = 2.0,
    ) -> None:
        self.num_joints = num_joints
        self.cutoff_hz = float(cutoff_hz)
        self.control_hz = float(control_hz)
        self.max_joint_speed_rad_s = float(max_joint_speed_rad_s)
        self._target = [0.0] * num_joints
        self._filtered = [0.0] * num_joints
        self._initialized = False
        self._alpha = self._compute_alpha()

    def _compute_alpha(self) -> float:
        # Discrete first-order LPF: y += a*(x-y), a = 1 - exp(-2*pi*fc/fs)
        if self.cutoff_hz <= 0.0:
            return 1.0
        if self.cutoff_hz >= 1000.0:
            # Official docs: cutoff >= 1 kHz disables filtering
            return 1.0
        return 1.0 - math.exp(-2.0 * math.pi * self.cutoff_hz / max(self.control_hz, 1.0))

    def set_params(
        self,
        cutoff_hz: Optional[float] = None,
        control_hz: Optional[float] = None,
        max_joint_speed_rad_s: Optional[float] = None,
    ) -> None:
        if cutoff_hz is not None:
            self.cutoff_hz = float(cutoff_hz)
        if control_hz is not None:
            self.control_hz = float(control_hz)
        if max_joint_speed_rad_s is not None:
            self.max_joint_speed_rad_s = float(max_joint_speed_rad_s)
        self._alpha = self._compute_alpha()

    @property
    def initialized(self) -> bool:
        return self._initialized

    def reset(self, positions: Optional[List[float]] = None) -> None:
        if positions is None:
            self._filtered = [0.0] * self.num_joints
            self._target = [0.0] * self.num_joints
            self._initialized = False
            return
        pos = list(positions[: self.num_joints])
        if len(pos) < self.num_joints:
            pos.extend([0.0] * (self.num_joints - len(pos)))
        self._filtered = [float(x) for x in pos]
        self._target = list(self._filtered)
        self._initialized = True

    def set_target(self, positions: List[float]) -> None:
        pos = list(positions[: self.num_joints])
        if len(pos) < self.num_joints:
            pos.extend([0.0] * (self.num_joints - len(pos)))
        self._target = [float(x) for x in pos]
        # Do NOT snap filtered←target. Official realtime filter seeds from
        # actual joint position via reset(); until then filter from zeros.
        if not self._initialized:
            self._initialized = True

    def step(self) -> List[float]:
        """Advance one control tick; return filtered command to send."""
        if not self._initialized:
            return list(self._filtered)

        dt = 1.0 / max(self.control_hz, 1.0)
        max_step = self.max_joint_speed_rad_s * dt
        a = self._alpha
        out: List[float] = [0.0] * self.num_joints
        for i in range(self.num_joints):
            # Low-pass toward latest target
            y = self._filtered[i] + a * (self._target[i] - self._filtered[i])
            # Slew-rate clamp relative to previous output
            dy = y - self._filtered[i]
            if dy > max_step:
                y = self._filtered[i] + max_step
            elif dy < -max_step:
                y = self._filtered[i] - max_step
            self._filtered[i] = y
            out[i] = y
        return out
