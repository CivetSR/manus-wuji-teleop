from __future__ import annotations

from wuji_hand_sim.sim_hand_worker import SimHandWorker


class FakeScene:
    def __init__(self) -> None:
        self.position = [0.0] * 20
        self.controls: list[list[float]] = []

    def read_joint_state(self) -> tuple[list[float], list[float]]:
        return list(self.position), [0.0] * 20

    def set_ctrl(self, position: list[float]) -> None:
        self.controls.append(list(position))


def test_sim_disarm_replaces_stale_target_and_stops_control_ticks() -> None:
    scene = FakeScene()
    worker = SimHandWorker("left", scene)  # type: ignore[arg-type]
    worker.connect()
    worker.set_enabled(True)
    worker.set_joint_target([0.5] * 20)
    worker._tick()
    assert scene.controls[-1] != [0.0] * 20

    scene.position = [0.2] * 20
    worker.set_enabled(False)
    sent_at_disarm = len(scene.controls)
    assert scene.controls[-1] == [0.2] * 20

    worker._tick()
    assert len(scene.controls) == sent_at_disarm
