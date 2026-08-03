from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
for path in (ROOT / "bridge" / "x86", ROOT / "bridge", ROOT / "sim"):
    sys.path.insert(0, str(path))
