from __future__ import annotations

import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = next(
    (p for p in [HERE, *HERE.parents] if (p / "dji_gimbal_cli.py").exists()),
    HERE.parents[2],
)
for path in (ROOT, HERE.parent):
    s = str(path)
    if s not in sys.path:
        sys.path.insert(0, s)
