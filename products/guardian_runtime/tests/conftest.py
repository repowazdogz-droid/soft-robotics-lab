"""Pytest conftest: ensure guardian_runtime is importable when run from package dir."""
import sys
from pathlib import Path

_root = Path(__file__).resolve().parent.parent.parent  # products/
if str(_root) not in sys.path:
    sys.path.insert(0, str(_root))
